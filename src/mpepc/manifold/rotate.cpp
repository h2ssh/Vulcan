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
 * Definition of RotateTaskManifold.
 */

#include "mpepc/manifold/rotate.h"
#include "core/angle_functions.h"
#include "core/motion_state.h"

namespace vulcan
{
namespace mpepc
{

RotateTaskManifold::RotateTaskManifold(float orientation)
: mode_(RotationMode::fixed_orientation)
, orientation_(wrap_to_pi(orientation))   // wrap to ensure the angle is within the [-pi,pi] range
{
}


RotateTaskManifold::RotateTaskManifold(RotationMode turnDirection) : mode_(turnDirection)
{
    assert((mode_ == RotationMode::turn_left) || (mode_ == RotationMode::turn_right));
}


void RotateTaskManifold::update(const planning_environment_t& env,
                                const std::vector<dynamic_object_trajectory_t>& trajectories)
{
    // In turn left or turn right mode, the goal orientation constantly moves away from the robot.
    if (mode_ == RotationMode::turn_left) {
        orientation_ = angle_sum(env.robotState.pose.theta, PI_2_F);
    } else if (mode_ == RotationMode::turn_right) {
        orientation_ = angle_diff(env.robotState.pose.theta, PI_2_F);
    }
    // Otherwise, the goal orientation is fixed

    // As turning in place is the desired behavior, the target pose is just the current pose with a different
    // orientation.
    targetPose_ = env.robotState.pose;
    targetPose_.theta = orientation_;
}


void RotateTaskManifold::calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory,
                                                                float timestep,
                                                                std::size_t stride,
                                                                std::vector<float>& rewards) const
{
    rewards.clear();   // ensure all rewards are being pushed to the correct index
    stride = std::max(stride, std::size_t(1));

    for (size_t index = stride; index < trajectory.size(); index += stride) {
        double initialError = angle_diff_abs(trajectory[index - stride].pose.theta, orientation_);
        double finalError = angle_diff_abs(trajectory[index].pose.theta, orientation_);

        rewards.push_back(finalError - initialError);
    }
}


double RotateTaskManifold::calculateHeuristicCost(const robot_trajectory_info_t& robotTrajectoryInfo) const
{
    // The rotate task has no heuristic
    return 0.0;
}


void RotateTaskManifold::sendDebugInfo(system::ModuleCommunicator& transmitter)
{
    // The rotate task has no debug info
}

}   // namespace mpepc
}   // namespace vulcan
