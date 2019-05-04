/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation.cpp
* \author   Collin Johnson
* 
* Definition of NavigationControlTask
*/

#include <planner/control/tasks/navigation.h>
#include <planner/control/tasks/pose.h>
#include <planner/control/state.h>

namespace vulcan
{
namespace planner
{
    
NavigationControlTask::NavigationControlTask(const pose_t& target, int32_t id, ControlTask* parent)
: ControlTask(id, parent)
, target_(target)
{
}


void NavigationControlTask::activate(const ControlState& state)
{
    // For the initial version, simply check that the target is in the LPM. If the target is in the LPM, create a 
    // PoseTask and sit back and wait. Otherwise, create no tasks which will cause failure when this task is executed
    if(state.map->isPoseInGrid(target_))
    {
        // Assign sub-tasks the same id as the parent task for easier error handling
        subtasks_.push_back(std::make_shared<detail::PoseTask>(target_, id(), this));
        completedSubtasks_.push_back(false);
    }
}


bool NavigationControlTask::hasSubTask(void) const 
{
    return !subtasks_.empty();
}


int NavigationControlTask::pushSubTasks(ControlTaskStack& stack) 
{
    for(auto& task : subtasks_)
    {
        stack.emplace_front(task);
    }
    
    return subtasks_.size();
}


void NavigationControlTask::childIsDone(ControlTask* child, ControlTaskProgress progress) 
{
    // When a child finishes, it needs to be marked as completed. The completed child should be the first child
    // not marked as completed when searching through the subtasks.
    for(std::size_t n = 0; n < subtasks_.size(); ++n)
    {
        if(subtasks_[n].get() == child)
        {
            // If the task has already been marked as completed, then the task was performed twice, which is also a
            // problem.
            if(completedSubtasks_[n])
            {
                std::cerr << "ERROR: NavigationControlTask: Child task was completed multiple times.\n";
            }
            completedSubtasks_[n] = true;
            break;
        }
        // If the completed tasks isn't the first subtask, something has gone wrong. Just output the error for now
        // and determine what to do about it later
        else if(!completedSubtasks_[n] && (subtasks_[n].get() != child))
        {
            std::cerr << "ERROR: NavigationControlTask: The completed child was not the first unfinished task. The task"
                << " stack has been corruped some how.\n";
            break;
        }
    }
}


ControlTaskResult NavigationControlTask::doExecute(const ControlState& state) 
{
    ControlTaskResult result;
    
    // If there were subtasks and all of them are complete, the task has succeeded
    if(!completedSubtasks_.empty() && 
        (std::find(completedSubtasks_.begin(), completedSubtasks_.end(), false) == completedSubtasks_.end()))
    {
        result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskProgress::completed);
    }
    // If there were no subtasks, the task couldn't be started because the target was invalid
    else if(completedSubtasks_.empty())
    {
        result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskError::target_invalid);
    }
    // Otherwise, one of the subtasks failed, so the target is unreachable
    else 
    {
        result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskError::target_unreachable);
    }
    
    return result;
}

} // namespace planner
} // namespace vulcan
