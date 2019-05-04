/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     differential_torque_plant.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of DifferentialTorqueDrive.
*/

#include <robot/model/differential_torque_plant.h>
#include <utils/timestamp.h>

namespace vulcan
{

namespace robot
{

DifferentialTorquePlant::DifferentialTorquePlant(const differential_torque_plant_params_t& params)
: params_(params)
{
}


motion_state_t DifferentialTorquePlant::nextState(const motion_state_t& state, const motion_command_t& command, float timestep)
{
    motion_state_t updatedState; // haveDistribution = haveWheels = false;
    
    velocity_t referenceVelocity(command.velocityCommand.linear, command.velocityCommand.angular);
    
    wheel_speeds_t currentWheelSpeeds(state.velocity,    params_.wheelbase);
    wheel_speeds_t desiredWheelSpeeds(referenceVelocity, params_.wheelbase);
    wheel_speeds_t wheelAccelerations;
    
    // compute torque (i.e. accelerations) from desired and current speed and saturation parameter
    wheelAccelerations.right = fmin(fabs(desiredWheelSpeeds.right - currentWheelSpeeds.right) / timestep, params_.wheelAccelMax);
    wheelAccelerations.right = copysign(wheelAccelerations.right, desiredWheelSpeeds.right - currentWheelSpeeds.right);
    
    wheelAccelerations.left  = fmin(fabs(desiredWheelSpeeds.left  - currentWheelSpeeds.left) / timestep,  params_.wheelAccelMax);
    wheelAccelerations.left  = copysign(wheelAccelerations.left,  desiredWheelSpeeds.left  - currentWheelSpeeds.left);
    
    // compute robot accelerations using the wheel accelerations
    velocity_t robotAccelerations = wheelAccelerations.toRobot(params_.wheelbase);
    updatedState.acceleration = acceleration_t(robotAccelerations.linear, robotAccelerations.angular); // convert to proper type

    // compute velocity using the wheel acceleration (cap with physical maximum speed of robot)
    wheel_speeds_t prevWheelSpeeds = currentWheelSpeeds;
    float temp_wheel_speed_right = currentWheelSpeeds.right + wheelAccelerations.right * timestep;
    float temp_wheel_speed_left = currentWheelSpeeds.left + wheelAccelerations.left * timestep;
    currentWheelSpeeds.right = fmin(fabs(temp_wheel_speed_right), params_.wheelVelMax);
    currentWheelSpeeds.right = copysign(currentWheelSpeeds.right, temp_wheel_speed_right);
    currentWheelSpeeds.left = fmin(fabs(temp_wheel_speed_left), params_.wheelVelMax);
    currentWheelSpeeds.left = copysign(currentWheelSpeeds.left, temp_wheel_speed_left);
    
    // compute average velocity.
    wheel_speeds_t averageWheelSpeeds;
    averageWheelSpeeds.right = (prevWheelSpeeds.right+currentWheelSpeeds.right)/2.0;
    averageWheelSpeeds.left = (prevWheelSpeeds.left+currentWheelSpeeds.left)/2.0;
    velocity_t averageVelocity = averageWheelSpeeds.toRobot(params_.wheelbase);
    
    // final velocity update
    updatedState.velocity = currentWheelSpeeds.toRobot(params_.wheelbase);
    
    // update pose using the average velocities
    updatedState.pose = turnDriveTurnIntegration(state.pose, averageVelocity, timestep);
    
    // update timestamp
    updatedState.timestamp = state.timestamp + utils::sec_to_usec(timestep);
    
    return updatedState;
}


DifferentialTorquePlant::wheel_speeds_t::wheel_speeds_t(const velocity_t& velocity, float wheelbase)
{
    right = velocity.linear + (velocity.angular * wheelbase / 2);
    left  = 2*velocity.linear - right;
}

velocity_t DifferentialTorquePlant::wheel_speeds_t::toRobot(float wheelbase) const
{
    return velocity_t((right + left) / 2,
                      (right - left) / wheelbase);
}


} // robot
} // vulcan
