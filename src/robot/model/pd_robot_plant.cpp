/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pd_robot_plant.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of PDRobotPlant.
*/

#include <robot/model/pd_robot_plant.h>
#include <utils/timestamp.h>

namespace vulcan
{

namespace robot
{

inline float saturate_value(float value, float saturation)
{
    return copysign(std::min(value, saturation), value);
}


PDRobotPlant::PDRobotPlant(const pd_robot_plant_params_t& params)
: params_(params)
{
}


motion_state_t PDRobotPlant::nextState(const motion_state_t& state, const motion_command_t& command, float timestep)
{
    motion_state_t updatedState;

    // error between commanded and current speed
    float linearError  = command.velocityCommand.linear  - state.velocity.linear;
    float angularError = command.velocityCommand.angular - state.velocity.angular;
    
    // compute and update acceleration
    float linearAcceleration  = linearError *params_.pGain + state.acceleration.linear *params_.dGain;
    float angularAcceleration = angularError*params_.pGain + state.acceleration.angular*params_.dGain;
    
    if(params_.useAccelerationSaturation)
    {
        linearAcceleration  = saturate_value(linearAcceleration,  params_.linearAccelSaturation);
        angularAcceleration = saturate_value(angularAcceleration, params_.angularAccelSaturation);
    }
    
    updatedState.acceleration.linear  = linearAcceleration;
    updatedState.acceleration.angular = angularAcceleration;

    // update pose based on average velocities
    float deltaLinearHalf  = 0.5 * timestep * linearAcceleration;
    float deltaAngularHalf = 0.5 * timestep * angularAcceleration;

    float meanLinear  = state.velocity.linear  + deltaLinearHalf;
    float meanAngular = state.velocity.angular + deltaAngularHalf;
    
    updatedState.pose = turnDriveTurnIntegration(state.pose, velocity_t(meanLinear, meanAngular), timestep); 
    
    // update velocities
    updatedState.velocity.linear  = meanLinear  + deltaLinearHalf;
    updatedState.velocity.angular = meanAngular + deltaAngularHalf;
    
    // update timestamp
    updatedState.timestamp = state.timestamp + utils::sec_to_usec(timestep);

    return updatedState;
}


} // namespace robot
} // namespace vulcan
