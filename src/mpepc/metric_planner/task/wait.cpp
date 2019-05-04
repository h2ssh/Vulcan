/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wait.cpp
* \author   Collin Johnson
* 
* Definition of WaitTask and WaitTaskManifold, which enable a program using just tasks to temporarily halt the robot
* without needing to send a separate message.
*/

#include <mpepc/metric_planner/task/wait.h>
#include <mpepc/manifold/wait.h>
#include <core/motion_state.h>
#include <utils/timestamp.h>
#include <cassert>

namespace vulcan
{
namespace mpepc
{
    
//////////////////// WaitTask implementation ////////////////////
    
WaitTask::WaitTask(int32_t durationMs)
: id_(utils::system_time_us())
, duration_(durationMs * 1000ll)
, startTime_(0)
{
    assert(duration_ >= 0);
}


bool WaitTask::setTaskParameters(const metric_planner_task_params_t& taskParams, 
                                 const task_manifold_builder_params_t& builderParams) 
{
    // There are no parameters needed, so we can always accept them
    return true;
}


std::unique_ptr<TaskManifold> WaitTask::createTaskManifold(void) 
{
    startTime_ = utils::system_time_us();
    return std::unique_ptr<TaskManifold>(new WaitTaskManifold());
}


bool WaitTask::isSafeToExecute(const ObstacleDistanceGrid& map) const 
{
    // It is always safe to stop
    return true;
}


bool WaitTask::isComplete(const motion_state_t& state) const 
{
    // If the task hasn't been started or the task isn't a timed wait, then we can't be complete
    if((duration_ == 0) || (startTime_ == 0))
    {
        return false;
    }
    
    // Otherwise, see if enough time has elapsed for the task to be completed
    return (utils::system_time_us() - startTime_ > duration_);
}

} // namespace mpepc
} // namespace vulcan
