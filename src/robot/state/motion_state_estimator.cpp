/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_state_estimator.cpp
* \author   Collin Johnson
* 
* Implementation of MotionStateEstimator.
*/

#include "robot/state/motion_state_estimator.h"
#include "core/motion_state.h"

namespace vulcan
{
namespace robot
{
    
motion_state_estimator_params_t::motion_state_estimator_params_t(const utils::ConfigFile& config, const utils::ConfigFile& robotConfig)
: poseEstimatorParams(config)
, velocityEstimatorParams(config)
, wheelsEstimatorParams(config)
, robotModelParams(robotConfig)
{
}


MotionStateEstimator::MotionStateEstimator(const motion_state_estimator_params_t& params)
: poseEstimator_(params.poseEstimatorParams)
, velocityEstimator_(params.velocityEstimatorParams)
, driveWheelEstimator_(params.wheelsEstimatorParams, params.robotModelParams)
{
}


void MotionStateEstimator::initialize(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<imu_data_t>    (&inputQueue_);
    communicator.subscribeTo<odometry_t>    (&inputQueue_);
    communicator.subscribeTo<encoder_data_t>(&inputQueue_);
    communicator.subscribeTo<commanded_joystick_t>   (&inputQueue_);
    communicator.subscribeTo<commanded_velocity_t>   (&inputQueue_);
    communicator.subscribeTo<pose_distribution_t>    (&inputQueue_);
}


void MotionStateEstimator::estimate(system::ModuleCommunicator& transmitter)
{
    if(!inputQueue_.isDataAvailable())
    {
        return;
    }
    
    auto input = inputQueue_.readInput();
    
    auto estimatedPose     = poseEstimator_.updateEstimate(input);
    auto estimatedVelocity = velocityEstimator_.updateEstimate(input, estimatedPose);
    auto estimatedWheels   = driveWheelEstimator_.updateEstimate(input, estimatedVelocity);
    
    motion_state_t updatedState(estimatedPose, estimatedVelocity, driveWheelEstimator_.getEstimatedAcceleration(), estimatedWheels);
    
    transmitter.sendMessage(updatedState);
}

}
}
