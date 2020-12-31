/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     rotate.cpp
 * \author   Collin Johnson
 *
 * Definition of RotateTask and RotateTaskManifold.
 */

#include "mpepc/metric_planner/task/rotate.h"
#include "core/angle_functions.h"
#include "core/motion_state.h"
#include "mpepc/manifold/rotate.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace mpepc
{

//////////////////////// RotateTask implementation /////////////////////////////////////

RotateTask::RotateTask(double goalOrientation, double maxCompletionError)
: id_(utils::system_time_us())
, mode_(RotationMode::fixed_orientation)
, orientation_(goalOrientation)
, maxError_(maxCompletionError)
{
    assert(maxCompletionError > 0.0);
}


RotateTask::RotateTask(RotationMode turnDirection) : id_(utils::system_time_us()), mode_(turnDirection)
{
    assert((mode_ == RotationMode::turn_left) || (mode_ == RotationMode::turn_right));
}


bool RotateTask::setTaskParameters(const metric_planner_task_params_t& taskParams,
                                   const task_manifold_builder_params_t& builderParams)
{
    // No rotate-specific parameters needed for now
    return true;
}


std::unique_ptr<TaskManifold> RotateTask::createTaskManifold(void)
{
    // For the fixed orientation mode, the orientation to move to is specified
    if (mode_ == RotationMode::fixed_orientation) {
        return std::unique_ptr<TaskManifold>(new RotateTaskManifold(orientation_));
    }
    // Otherwise, just send along the user-specified rotation mode
    else {
        return std::unique_ptr<TaskManifold>(new RotateTaskManifold(mode_));
    }
}


bool RotateTask::isSafeToExecute(const ObstacleDistanceGrid& map) const
{
    // For now, assume that the robot can always try to turn-in-place. It should refuse to hit walls in MPEPC, so this
    // assumption is valid for now
    return true;
}


bool RotateTask::isComplete(const motion_state_t& state) const
{
    // In fixed orientation mode, see if the robot is within max error of the orientation
    if (mode_ == RotationMode::fixed_orientation) {
        return angle_diff_abs(state.pose.theta, orientation_) < maxError_;
    }

    // In turn mode, the task is never completed.
    return false;
}

}   // namespace mpepc
}   // namespace vulcan
