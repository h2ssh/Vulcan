/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     differential_motors_plant.cpp
 * \author   Jong Jin Park and Paul Foster
 *
 * Definition of DifferentialMotorsPlant.
 */

#include "robot/model/differential_motors_plant.h"
#include "utils/timestamp.h"

namespace vulcan
{

namespace robot
{

DifferentialMotorsPlant::DifferentialMotorsPlant(const differential_motors_plant_params_t& params) : params_(params)
{
}


motion_state_t DifferentialMotorsPlant::nextState(const motion_state_t& currentState,
                                                  const motion_command_t& command,
                                                  float timestep)
{
    differential_drive_wheels_t updatedWheelState =
      nextWheelState(currentState.differentialWheels, command.joystickCommand, timestep);

    // convert motor speeds to linear and angular velocities
    velocity_t updatedVelocity((updatedWheelState.rightWheel.speed + updatedWheelState.leftWheel.speed) * 0.5,
                               (updatedWheelState.rightWheel.speed - updatedWheelState.leftWheel.speed)
                                 / params_.wheelbase);

    // compute average velocities for pose integration
    velocity_t averageVelocity((currentState.velocity.linear + updatedVelocity.linear) * 0.5,
                               (currentState.velocity.angular + updatedVelocity.angular) * 0.5);

    // integrate velocities to get the pose update
    pose_t updatedPose = turnDriveTurnIntegration(currentState.pose, averageVelocity, timestep);
    updatedPose.timestamp = currentState.timestamp + utils::sec_to_usec(timestep);   // update timestamp

    // record acceleration over the time step. Note that this is half a step behind the velocity estimate.
    acceleration_t updatedAcceleration((updatedVelocity.linear - currentState.velocity.linear) / timestep,
                                       (updatedVelocity.angular - currentState.velocity.angular) / timestep);

    return motion_state_t(updatedPose, updatedVelocity, updatedAcceleration, updatedWheelState);
}


differential_drive_wheels_t DifferentialMotorsPlant::nextWheelState(const differential_drive_wheels_t& currentState,
                                                                    const joystick_command_t& joystickCommand,
                                                                    float timestep)
{
    differential_drive_wheels_t updatedState;

    // convert given joystick commands to right and left motor commands
    double turnRate = (joystickCommand.forward == 0)
      ? params_.turnRateInPlace
      : params_.turnRateBase * (1 - params_.turnReductionRate * fabs(joystickCommand.forward));
    double rightMotorCommand = joystickCommand.forward + turnRate * joystickCommand.left;
    double leftMotorCommand = joystickCommand.forward - turnRate * joystickCommand.left;

    // update motion state based on the motor model
    updatedState.rightWheel = motor_model(currentState.rightWheel,
                                          rightMotorCommand,
                                          timestep,
                                          params_.motorMu,
                                          params_.motorBeta,
                                          params_.motorGamma,
                                          params_.motorAlpha,
                                          nullptr);
    updatedState.leftWheel = motor_model(currentState.leftWheel,
                                         leftMotorCommand,
                                         timestep,
                                         params_.motorMu,
                                         params_.motorBeta,
                                         params_.motorGamma,
                                         params_.motorAlpha,
                                         nullptr);

    return updatedState;
}

}   // namespace robot
}   // namespace vulcan
