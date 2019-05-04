/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_model.h
* \author   Collin Johnson
*
* Definition of InverseMotionModel interface and motion_model_data_t POD.
*/

#ifndef ROBOT_MODEL_MOTION_MODEL_H
#define ROBOT_MODEL_MOTION_MODEL_H

#include <core/pose_distribution.h>
#include <vector>

namespace vulcan
{
struct imu_data_t;
struct odometry_t;

namespace robot
{

struct commanded_velocity_t;

/**
* motion_model_data_t holds possible data needed a motion model. Each type is optional and
* a motion model may not consume everything.
*/
struct motion_model_data_t
{
    const std::vector<imu_data_t>*  imu;
    const odometry_t*               odometry;
    const std::vector<commanded_velocity_t>* commands;

    motion_model_data_t(void)
    : imu(0)
    , odometry(0)
    , commands(0)
    {
    }
};

/**
* InverseMotionModel
*/
class InverseMotionModel
{
public:

    virtual ~InverseMotionModel(void) { }

    /**
    * predictPose implements the prediction step of a KF/EKF, finding the predicted pose based on
    * the control input. The previousPose is the result from a previous filtering calculation.
    *
    * Both the data from the previous timestemp and the current timestep are provided. Some motion models,
    * like odometry, fake a control input by looking at the difference between odometry measurements at
    * consecutive timesteps.
    *
    * \param    previousPose            Pose from a previous filtering operation
    * \param    previousData            Data used for the last pose update
    * \param    currentData             Data for the current update
    * \return   Pose distribution determined by applying the new control signal the previous pose.
    */
    virtual pose_distribution_t predictPose(const pose_distribution_t& previousPose,
                                            const motion_model_data_t& previousData,
                                            const motion_model_data_t& currentData) = 0;
};

}
}

#endif // ROBOT_MODEL_MOTION_MODEL_H
