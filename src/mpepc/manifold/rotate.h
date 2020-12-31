/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rotate.h
* \author   Collin Johnson
* 
* Declaration of RotateTaskManifold that has the robot rotate continuously in the same spot.
*/

#ifndef MPEPC_MANIFOLDS_ROTATE_H
#define MPEPC_MANIFOLDS_ROTATE_H

#include "mpepc/types.h"
#include "mpepc/manifold/task_manifold.h"
#include "core/pose.h"

namespace vulcan
{
namespace mpepc
{

/**
* RotateTaskManifold is a manifold that encourages the robot to turn-in-place. The progress term is solely related to
* the difference between the desired and current robot orientation. When the manifold is in fixed_orientation mode, the
* desired orientation never changes. When in turn_left or turn_right mode, every time update is called, the orientation
* will be updated to keep it pi/2 to the left/right of the robot, thereby always encouraging turn-in-place behavior.
*/
class RotateTaskManifold : public TaskManifold
{
public:
    
    /**
    * Constructor for RotateTaskManifold.
    * 
    * Create a manifold for a task that has the robot turn to a specified orientation.
    * 
    * \param    orientation         Desired orientation for the robot to reach
    */
    explicit RotateTaskManifold(double orientation);
    
    /**
    * Constructor for RotateTaskManifold.
    * 
    * Create a manifold for a task that has the robot turn either left or right forever. This type of task will never
    * be completed.
    * 
    * \param    turnDirection           Direction to turn
    * \pre  turnDirection == RotationMode::turn_left || turnDirection == RotationMode::turn_right
    */
    explicit RotateTaskManifold(RotationMode turnDirection);
    
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
    
    RotationMode mode_;
    double orientation_;
    pose_t targetPose_; // if the robot were to turn perfect in-place, where would it end up?s
};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_MANIFOLDS_ROTATE_H
