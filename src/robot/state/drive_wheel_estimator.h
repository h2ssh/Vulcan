/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     drive_wheel_estimator.h
* \author   Collin Johnson
*
* Definition of DriveWheelEstimator.
*/

#ifndef ROBOT_STATE_DRIVE_WHEEL_ESTIMATOR_H
#define ROBOT_STATE_DRIVE_WHEEL_ESTIMATOR_H

#include <robot/state/motion_state_input.h>
#include <core/drive_wheel.h>
#include <core/velocity.h>
#include <robot/model/params.h>

namespace vulcan
{
namespace robot
{

/**
* drive_wheel_estimator_params_t
*/
struct drive_wheel_estimator_params_t
{
    // pre-computed stationary Kalman gain
    float filterSpeedGain      = 0.25;
    float filterMotorAccelGain = 0.0968;

    drive_wheel_estimator_params_t(const utils::ConfigFile& config);
};

/**
* DriveWheelEstimator
*/
class DriveWheelEstimator
{
public:

    /**
    * Constructor for DriveWheelEstimator.
    *
    * \param    params          Parameters for controlling the estimator
    */
    DriveWheelEstimator(const drive_wheel_estimator_params_t& params, const differential_motors_plant_params_t& robotParams);

    /**
    * updateEstimate
    */
    differential_drive_wheels_t updateEstimate(const motion_state_input_t& input, const velocity_t& estimatedVelocity);

    acceleration_t getEstimatedAcceleration(void) { return acceleration_; };

private:

    struct wheel_speeds_observation_t
    {
        int64_t timestamp = 0;

        double leftWheel = 0.0;
        double rightWheel = 0.0;
        double time = 0.0;
    };

    drive_wheel_estimator_params_t     params_;
    differential_motors_plant_params_t robotParams_;

    drive_wheel_t           previousLeftWheel_;
    drive_wheel_t           previousRightWheel_;
    velocity_t              previousVelocity_;
    encoder_data_t previousEncoders_;

    acceleration_t          acceleration_;

    bool encodersInitialized_;
    bool velocityInitialized_;

    wheel_speeds_observation_t calculateWheelObservation(const motion_state_input_t& input, const velocity_t& estimatedVelocity);
    drive_wheel_t              applyFilterToWheelSpeed(const drive_wheel_t& previousState, double observedSpeed, double motorCommand, double timeDelta, int64_t timestamp, double* friction);
};

}
}

#endif // ROBOT_STATE_DRIVE_WHEEL_ESTIMATOR_H
