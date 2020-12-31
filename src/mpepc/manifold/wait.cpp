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
* Definition of WaitTaskManifold that handles the WaitTask of temporarily halting the robot.
*/

#include "mpepc/manifold/wait.h"
#include "mpepc/types.h"
#include "core/angle_functions.h"
#include "core/motion_state.h"

namespace vulcan 
{
namespace mpepc
{

WaitTaskManifold::WaitTaskManifold(void)
{
}


void WaitTaskManifold::update(const planning_environment_t& env,
                              const std::vector<dynamic_object_trajectory_t>& trajectories)
{
    // When updating, the current target pose needs to be changed to the robot's current pose
    targetPose_ = env.robotState.pose;
}


void WaitTaskManifold::calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory, 
                                                              float timestep, 
                                                              std::size_t stride, 
                                                              std::vector<float>& rewards) const 
{
    rewards.clear();  // ensure all rewards are being pushed to the correct index
    stride = std::max(stride, std::size_t(1));
    
    timestep *= stride;
    
    for(size_t index = stride; index < trajectory.size(); index += stride)
    {
        double orientationError = angle_diff_abs(trajectory[index].pose.theta, targetPose_.theta);
        double positionError = distance_between_points(trajectory[index].pose.toPoint(), targetPose_.toPoint());
        
        rewards.push_back(orientationError + positionError - timestep);
    }
}


double WaitTaskManifold::calculateHeuristicCost(const robot_trajectory_info_t& robotTrajectoryInfo) const 
{
    // No heuristic code
    return 0.0;
}


void WaitTaskManifold::sendDebugInfo(system::ModuleCommunicator& transmitter) 
{
    // There is no debug info
}

} // namespace mpepc
} // namespace vulcan
