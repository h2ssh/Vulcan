/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_state_estimator.h
* \author   Collin Johnson
* 
* Definition of MotionStateEstimator.
*/

#ifndef ROBOT_STATE_MOTION_STATE_ESTIMATOR_H
#define ROBOT_STATE_MOTION_STATE_ESTIMATOR_H

#include "robot/state/state_estimator.h"
#include "robot/state/motion_state_input.h"
#include "robot/state/pose_estimator.h"
#include "robot/state/velocity_estimator.h"
#include "robot/state/drive_wheel_estimator.h"

namespace vulcan
{
namespace robot
{
    
const std::string kMotionStateEstimatorType("motion_state");
    
/**
* motion_state_estimator_params_t
*/
struct motion_state_estimator_params_t
{
    pose_estimator_params_t            poseEstimatorParams;
    velocity_estimator_params_t        velocityEstimatorParams;
    drive_wheel_estimator_params_t     wheelsEstimatorParams;
    differential_motors_plant_params_t robotModelParams;
    
    motion_state_estimator_params_t(const utils::ConfigFile& config, const utils::ConfigFile& robotConfig);
};

/**
* MotionStateEstimator
*/
class MotionStateEstimator : public StateEstimator
{
public:
    
    /**
    * Constructor for MotionStateEstimator.
    * 
    * \param    params          Parameters controlling the behavior of the state estimator
    */
    MotionStateEstimator(const motion_state_estimator_params_t& params);
    
    // StateEstimator interface
    virtual void initialize(system::ModuleCommunicator& communicator);
    virtual void estimate  (system::ModuleCommunicator& transmitter);

private:

    MotionStateInputQueue inputQueue_;
    PoseEstimator         poseEstimator_;
    VelocityEstimator     velocityEstimator_;
    DriveWheelEstimator   driveWheelEstimator_;
};

}
}

#endif // ROBOT_STATE_MOTION_STATE_ESTIMATOR_H
