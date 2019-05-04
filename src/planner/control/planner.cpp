/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     planner.cpp
* \author   Collin Johnson
* 
* Definition of ControlPlanner.
*/

#include <planner/control/planner.h>
#include <planner/control/command.h>
#include <planner/control/state.h>
#include <iostream>

namespace vulcan
{
namespace planner
{
    
ControlPlanner::ControlPlanner(void)
{
}


void ControlPlanner::assignTask(const ControlCommand& command, const ControlState& state)
{   
    std::cout << "INFO:ControlPlanner: Starting to execute command from " << command.source() << " issued at "
        << command.timestamp() << "\nClearing " << taskStack_.size() << " previous tasks.\n";
        
    // Clear out any previous tasks that remained
    taskStack_.clear();
    
    // Activate the task and push any required subtasks
    currentTask_ = command.task();    
    activateTask(*currentTask_, state);
    
    // Handle the current task after all subtasks have completed
    taskStack_.push_back(currentTask_);
}


ControlTaskResult ControlPlanner::executeTask(const ControlState& state)
{
    // If there are no tasks, then currently waiting for a new one
    if(taskStack_.empty())
    {
        ControlTaskResult result;
        result.status = ControlTaskStatus(state.pose.timestamp(), kInvalidTaskStatusId, ControlTaskProgress::waiting);
    }
    
    // Execute the task, providing the most recent state
    auto& task = taskStack_.front();
    auto result = task->execute(state);
    
    // If the task has finished, then pop it off the stack, activate the next task, if available
    if((result.status.progress() == ControlTaskProgress::completed) 
        || (result.status.progress() == ControlTaskProgress::failed))
    {
        taskStack_.pop_front();
        
        if(!taskStack_.empty())
        {
            activateTask(*(taskStack_.front()), state);
        }
    }
    
    return result;
}


bool ControlPlanner::haveTask(void) const
{
    return !taskStack_.empty();
}


void ControlPlanner::activateTask(ControlTask& task, const ControlState& state)
{
    task.activate(state);
    
    if(task.hasSubTask())
    {
        int numSubtasks = task.pushSubTasks(taskStack_);
        
        std::cout << "Added " << numSubtasks << " subtasks.\n";
    }
    
}

} // namespace planner
} // namespace vulcan
