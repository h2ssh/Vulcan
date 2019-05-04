/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     velocity_estimator.h
* \author   Collin Johnson
* 
* Definition of VelocityEstimator.
*/

#ifndef ROBOT_STATE_VELOCITY_ESTIMATOR_H
#define ROBOT_STATE_VELOCITY_ESTIMATOR_H

#include <robot/state/motion_state_input.h>
#include <core/velocity.h>
#include <deque>

namespace vulcan
{
namespace robot
{
    
/**
* velocity_estimator_params_t defines the parameters controlling the velocity estimator. The parameters are:
* 
*   [VelocityEstimatorParameters]
*   sliding_window_size_ms = size of the sliding window to use when estimating velocity by differentiating poses in milliseconds
*   odometry_filter_gain   = Kalman filter gain to use in the odometry velocity filter
*/
struct velocity_estimator_params_t
{
    int   slidingWindowSizeMs;
    float odometryFilterGain;
    
    velocity_estimator_params_t(const utils::ConfigFile& config);
};

/**
* VelocityEstimator estimates the velocity of the robot using one of two methods:
* 
* 1) Determine the amount of translation and rotation the robot has performed within a fixed time window and then
*    divide by the time elapsed to get a guess on the current velocity.
* 2) Directly read the robot's velocity from odometry data.
* 3) If IMU data is available, use the current reading from the gyro for the angular velocity.
*/
class VelocityEstimator
{
public:
    
    /**
    * Constructor for VelocityEstimator.
    * 
    * \param    params          Parameters for the velocity estimator
    */
    VelocityEstimator(const velocity_estimator_params_t& params);
    
    /**
    * updateEstimate updates the current estimate of the velocity using one of the two methods described above.
    * 
    * \param    input           Current data available
    * \param    estimatedPose   Pose estimated by the PoseEstimator using the current input
    * \return   An estimate of the robot's current velocity.
    */
    velocity_t updateEstimate(const motion_state_input_t& input, const pose_distribution_t& estimatedPose);
    
private:
    
    const velocity_estimator_params_t params_;
    bool                              initialized_;
    
    std::deque<pose_distribution_t> poseWindow_;
    odometry_t             lastOdometry_;
    velocity_t                      previousVelocityEstimate_;
    
    void       updatePoseWindow     (const pose_distribution_t& newPose);
    velocity_t slidingWindowVelocity(void);
    velocity_t odometryVelocity     (const std::vector<odometry_t>& odometry);
};

}
}

#endif // ROBOT_STATE_VELOCITY_ESTIMATOR_H
