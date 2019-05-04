/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.cpp
* \author   Collin Johnson
* 
* Definition of PoseTask.
*/

#include <planner/control/tasks/pose.h>
#include <planner/control/state.h>
#include <mpepc/metric_planner/task/navigation.h>

namespace vulcan
{
namespace planner
{
namespace detail
{
    
PoseTask::PoseTask(const pose_t& target, int32_t id, ControlTask* parent)
: ControlTask(id, parent)
, target_(target)
{    
}
    
// ControlTask interface
void PoseTask::activate(const ControlState& state)
{
    // No checks are need for activation
}


bool PoseTask::hasSubTask(void) const 
{
    // The PoseTask never has sub-tasks. It is a low-level task.
    return false;
}


int PoseTask::pushSubTasks(ControlTaskStack& stack) 
{
    // The PoseTask never has sub-tasks.
    return 0;
}


void PoseTask::childIsDone(ControlTask* child, ControlTaskProgress progress) 
{
    // The PoseTask never has children, so something is broken if this method is called
    std::cerr << "ERROR: PoseTask: Received a childIsDone message, but task doesn't have any children.\n";
}


ControlTaskResult PoseTask::doExecute(const ControlState& state) 
{
    ControlTaskResult result;
    
    // If no task has been sent yet, the create it and send it off.
    if(!mpepcTask_)
    {
        mpepcTask_ = std::make_shared<mpepc::NavigationTask>(target_);
        result.task = mpepcTask_;
        result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskProgress::executing);
    }
    // The task has been sent, so the status from the metric_planner determines the task's progress
    else if(state.metricPlannerStatus)
    {
        // If the pose was reached, then this task is finished
        if(state.metricPlannerStatus->status == mpepc::SUCCESS_REACHED_POSE)
        {
            result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskProgress::completed);
        }
        // If the target cannot be assigned, then it is invalid
        else if(state.metricPlannerStatus->status == mpepc::FAILURE_CANNOT_ASSIGN_TASK)
        {
            result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskError::target_invalid);
        }
        // If the target cannot be reached or the a solution cannot be found, then the target is unreachable
        else if((state.metricPlannerStatus->status == mpepc::FAILURE_UNABLE_TO_PROGRESS) ||
            (state.metricPlannerStatus->status == mpepc::FAILURE_CANNOT_FIND_SOLUTION))
        {
            result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskError::target_unreachable);
        }
        // In all other cases, adequate progress is being made
        else
        {
            result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskProgress::executing);
        }
    }
    // If there isn't a status update, then the task is happily executing
    else
    {
        result.status = ControlTaskStatus(state.pose.timestamp(), id(), ControlTaskProgress::executing);
    }
    
    return result;
}

} // namespace detail
} // namespace planner
} // namespace vulcan
