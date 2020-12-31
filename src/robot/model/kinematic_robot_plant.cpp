/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     kinematic_robot_plant.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of KinematicRobotPlant.
*/

#include "robot/model/kinematic_robot_plant.h"
#include "core/velocity.h"
#include "utils/timestamp.h"

namespace vulcan
{

namespace robot
{

KinematicRobotPlant::KinematicRobotPlant(void)
{
    // Nothing needed for initialization
}

motion_state_t KinematicRobotPlant::nextState(const motion_state_t& state, const motion_command_t& command, float timestep)
{
    velocity_t newVelocity(command.velocityCommand.linear, command.velocityCommand.angular);
    pose_t     newPose(turnDriveTurnIntegration(state.pose, newVelocity, timestep));
    newPose.timestamp = state.timestamp + utils::sec_to_usec(timestep);
    
    return motion_state_t(newPose, newVelocity);
}

} // robot
} // vulcan