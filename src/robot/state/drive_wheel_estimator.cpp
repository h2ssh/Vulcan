/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     drive_wheel_estimator.cpp
* \author   Collin Johnson
* 
* Implementation of DriveWheelEstimator.
*/

#include <robot/state/drive_wheel_estimator.h>
#include <robot/model/motor_model.h>
#include <utils/config_file.h>
#include <utils/timestamp.h>
#include <cassert>

namespace vulcan
{
namespace robot
{
    
const std::string kDriveWheelHeading("DriveWheelEstimatorParameters");
const std::string kSpeedGainKey     ("speed_filter_gain");
const std::string kAccelGainKey     ("motor_accel_filter_gain");
    
struct wheel_speeds_t
{
    double leftWheel;
    double rightWheel;
    
    wheel_speeds_t(double linear, double angular, double wheelbase)
    : leftWheel(linear  - 0.5*wheelbase*angular)
    , rightWheel(linear + 0.5*wheelbase*angular)
    {
    }
};

struct motor_command_t
{
    double leftMotor;
    double rightMotor;
    
    motor_command_t(void)
    : leftMotor(0.0)
    , rightMotor(0.0)
    {
    }
    
    motor_command_t(const commanded_velocity_t& commandedVelocity, const differential_motors_plant_params_t& robotParams)
    {
        wheel_speeds_t desiredSpeeds(commandedVelocity.linearVelocity, commandedVelocity.angularVelocity, robotParams.wheelbase);
        
        leftMotor  = convert_speed_to_command(desiredSpeeds.leftWheel,  robotParams);
        rightMotor = convert_speed_to_command(desiredSpeeds.rightWheel, robotParams); 
    }
        
    motor_command_t(const commanded_joystick_t& commandedJoystick, const differential_motors_plant_params_t& robotParams)
    {
        float gain = compute_rotation_gain(commandedJoystick.forward, robotParams);
        
        leftMotor  = static_cast<double>(commandedJoystick.forward) - gain*commandedJoystick.left;
        rightMotor = static_cast<double>(commandedJoystick.forward) + gain*commandedJoystick.left;
    }
    
    double convert_speed_to_command(double speed, const differential_motors_plant_params_t& robotParams)
    {
        double command = 0.0;

        if(std::abs(speed) > 0.005)
        {
            command = robotParams.motorBeta/robotParams.motorAlpha*speed + std::copysign(robotParams.motorMu*robotParams.motorGamma/robotParams.motorAlpha, speed);
        }

        return command;
    }

    float compute_rotation_gain(int16_t forwardJoystickPosition, const differential_motors_plant_params_t& robotParams)
    {
        return (forwardJoystickPosition == 0) ? robotParams.turnRateInPlace : robotParams.turnRateBase * (1.0 - robotParams.turnReductionRate*abs(forwardJoystickPosition));
    }
    
};


drive_wheel_estimator_params_t::drive_wheel_estimator_params_t(const utils::ConfigFile& config)
: filterSpeedGain(config.getValueAsFloat(kDriveWheelHeading, kSpeedGainKey))
, filterMotorAccelGain(config.getValueAsFloat(kDriveWheelHeading, kAccelGainKey))
{
    assert(filterSpeedGain > 0.0f);
    assert(filterMotorAccelGain > 0.0f);
}


DriveWheelEstimator::DriveWheelEstimator(const drive_wheel_estimator_params_t& params, const differential_motors_plant_params_t& robotParams)
: params_(params)
, robotParams_(robotParams)
, encodersInitialized_(false)
, velocityInitialized_(false)
{
}


differential_drive_wheels_t DriveWheelEstimator::updateEstimate(const motion_state_input_t& input, const velocity_t& estimatedVelocity)
{
    // observation
    wheel_speeds_observation_t observation = calculateWheelObservation(input, estimatedVelocity);
    
    // commands
    // If there are no commands available, just assume that the current velocity is the desired velocity
    motor_command_t motorCommands;
    if(!input.commandedJoysticks.empty())
    {
        motorCommands = motor_command_t(input.commandedJoysticks.back(), robotParams_);
    }
    else if(!input.commandedVelocities.empty())
    {
        motorCommands = motor_command_t(input.commandedVelocities.back(), robotParams_);
    }
    else
    {
        auto fakeCommand = commanded_velocity_t(estimatedVelocity.linear, estimatedVelocity.angular, estimatedVelocity.timestamp);
        motorCommands = motor_command_t(fakeCommand, robotParams_);
    }

    // update
    int64_t previousWheelTimestamp = previousLeftWheel_.timestamp;
    if(previousLeftWheel_.timestamp != previousRightWheel_.timestamp)
    {
        std::cout<<"WARNING!!: DriveWheelEstimator: Inconsistent timestamp between the right and the left wheel.\n";
    }
    
    // left wheel
    double leftWheelFriction;
    previousLeftWheel_     = applyFilterToWheelSpeed(previousLeftWheel_,  observation.leftWheel,  motorCommands.leftMotor,  observation.time, observation.timestamp, &leftWheelFriction);
    float  leftWheelAccel  = previousLeftWheel_.motorAccel + leftWheelFriction;
    
    // right wheel
    double rightWheelFriction;
    previousRightWheel_    = applyFilterToWheelSpeed(previousRightWheel_, observation.rightWheel, motorCommands.rightMotor, observation.time, observation.timestamp, &rightWheelFriction);
    float  rightWheelAccel = previousRightWheel_.motorAccel + rightWheelFriction;
    
    int64_t timeDelta = previousRightWheel_.timestamp - previousWheelTimestamp;
    acceleration_ = acceleration_t( (rightWheelAccel + leftWheelAccel) / 2.0,
                                    (rightWheelAccel - leftWheelAccel) / robotParams_.wheelbase,
                                    previousWheelTimestamp + (timeDelta / 2) );
    
    return {previousLeftWheel_, previousRightWheel_};
}


DriveWheelEstimator::wheel_speeds_observation_t DriveWheelEstimator::calculateWheelObservation(const motion_state_input_t& input, const velocity_t& estimatedVelocity)
{
    wheel_speeds_observation_t observation;
    
    if(!input.encoders.empty())
    {
        if(!encodersInitialized_)
        {
            previousEncoders_    = input.encoders.front();
            encodersInitialized_ = true;
        }
        
        double totalLeftDelta  = 0.0;
        double totalRightDelta = 0.0;
        for(const auto& encoder : input.encoders)
        {
            totalLeftDelta  += encoder.deltaLeftWheel;
            totalRightDelta += encoder.deltaRightWheel;
        }
        
        observation.timestamp = input.encoders.back().timestamp;
        
        observation.time  = utils::usec_to_sec(input.encoders.back().timestamp - previousEncoders_.timestamp);
        previousEncoders_ = input.encoders.back();
        
        if(observation.time > 0.0)
        {
            observation.leftWheel  = totalLeftDelta  / observation.time;
            observation.rightWheel = totalRightDelta / observation.time;
        }
    }
    else
    {
        if(!velocityInitialized_)
        {
            previousVelocity_    = estimatedVelocity;
            velocityInitialized_ = true;
        }
        
        auto speeds = wheel_speeds_t(estimatedVelocity.linear, estimatedVelocity.angular, robotParams_.wheelbase);
        observation.leftWheel   = speeds.leftWheel;
        observation.rightWheel  = speeds.rightWheel;
        observation.time        = utils::usec_to_sec(estimatedVelocity.timestamp - previousVelocity_.timestamp);

        observation.timestamp = previousVelocity_.timestamp + (estimatedVelocity.timestamp - previousVelocity_.timestamp)/2;
    }
    
    // detect wheel slip and switch to using IMU measurement if it does
    if(!input.imu.empty())
    {
        double imuAngularVelocity     = input.imu.back().rotationalVelocity[IMU_YAW_INDEX];
        double encoderAngularVelocity = (observation.rightWheel - observation.leftWheel) / robotParams_.wheelbase;
        
        if(fabs(imuAngularVelocity - encoderAngularVelocity) > 0.1)
        {
            std::cout<<"Warning!!: DriveWheelEstimator: Possible wheel slip detected: RotationalVelocity (rad/s): "<<"IMU: "<<imuAngularVelocity << ", Encoder: "<<encoderAngularVelocity<<'\n';
            
            double encoderLinearVelocity = (observation.rightWheel + observation.leftWheel) / 2.0;
            auto speeds = wheel_speeds_t(encoderLinearVelocity, imuAngularVelocity, robotParams_.wheelbase);
            observation.leftWheel  = speeds.leftWheel;
            observation.rightWheel = speeds.rightWheel;
//             observation.time       = input.imu.back().timestamp;
        }
    }
    
    previousVelocity_ = estimatedVelocity;
    
    return observation;
}


drive_wheel_t DriveWheelEstimator::applyFilterToWheelSpeed(const drive_wheel_t& previousState, double observedSpeed, double motorCommand, double timeDelta, int64_t timestamp, double* friction)
{
    // Kalman prediction
    // update motion state based on the motor model. right and left driveWheelState
    auto predictedWheelState = motor_model(previousState, motorCommand, timeDelta, robotParams_.motorMu, robotParams_.motorBeta, robotParams_.motorGamma, robotParams_.motorAlpha, friction);
    
    // Kalman update with pre-computed stationary gain
    float innovation = observedSpeed - predictedWheelState.speed;
    
    return drive_wheel_t(timestamp,
                         predictedWheelState.speed      + params_.filterSpeedGain*innovation,
                         predictedWheelState.motorAccel + params_.filterMotorAccelGain*innovation);
}

} // namespace robot
} // namespace vulcan
