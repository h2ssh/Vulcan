/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wait.h
* \author   Collin Johnson 
* 
* Declaration of WaitTaskManifold that handles the WaitTask of temporarily halting the robot.
*/

#ifndef MPEPC_MANIFOLDS_WAIT_H
#define MPEPC_MANIFOLDS_WAIT_H

#include <mpepc/manifold/task_manifold.h>
#include <core/pose.h>

namespace vulcan
{
namespace mpepc
{

class ObstacleDistanceGrid;

/**
* WaitTaskManifold is a simple manifold that has no progress for anything other than remaining at the robot's current
* pose. The reward is directly proportional to the amount of time the robot spends at the target pose for the duration
* of the trajectory.
* 
* The reward specifically is:
* 
*   reward = dist_from_target - timestep
* 
* Thus, if the dist from target is 0, then the maximum reward is gained. The distance from target is the sum of the
* position distance and orientation distance of the pose along the trajectory.
* 
* Each time the manifold is updated, the robot's current pose will be set as the target.
*/
class WaitTaskManifold : public TaskManifold
{
public:
    
    /**
    * Constructor for WaitTaskManifold.
    * 
    * Create a manifold that has the robot wait at its current pose.
    */
    WaitTaskManifold(void);
    
    // TaskManifold interface
    pose_t target(void) const override { return targetPose_; }
    void update(const planning_environment_t& env,
                const std::vector<dynamic_object_trajectory_t>& trajectories) override;
    void calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory, 
                                                float timestep, 
                                                std::size_t stride, 
                                                std::vector<float>& rewards) const override;
    double calculateHeuristicCost(const robot_trajectory_info_t& robotTrajectoryInfo) const override;
    void sendDebugInfo(system::ModuleCommunicator& transmitter) override;

private:
    
    pose_t targetPose_; // the target is always the current pose of the robot
};

} // namespace mpepc
} // namespace vulcan 

#endif // MPEPC_MANIFOLDS_WAIT_H
