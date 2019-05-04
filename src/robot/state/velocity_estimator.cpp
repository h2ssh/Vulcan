/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     velocity_estimator.cpp
* \author   Collin Johnson
* 
* Implementation of VelocityEstimator.
*/

#include <robot/state/velocity_estimator.h>
#include <utils/config_file.h>
#include <utils/timestamp.h>

namespace vulcan
{
namespace robot
{
    
const std::string kVelocityEstimatorHeading("VelocityEstimatorParameters");
const std::string kWindowSizeKey           ("sliding_window_size_ms");
const std::string kFilterGainKey           ("odometry_filter_gain");

velocity_estimator_params_t::velocity_estimator_params_t(const utils::ConfigFile& config)
: slidingWindowSizeMs(config.getValueAsInt32(kVelocityEstimatorHeading, kWindowSizeKey))
, odometryFilterGain(config.getValueAsFloat(kVelocityEstimatorHeading, kFilterGainKey))
{
}


VelocityEstimator::VelocityEstimator(const velocity_estimator_params_t& params)
: params_(params)
, initialized_(false)
{
}


velocity_t VelocityEstimator::updateEstimate(const motion_state_input_t& input, const pose_distribution_t& estimatedPose)
{
    updatePoseWindow(estimatedPose);
    
    auto estimate = input.odometry.empty() ? slidingWindowVelocity() : odometryVelocity(input.odometry);
    
    // If there is IMU data, then use the most recent angular velocity measurement directly. It will be a better estimate
    // than either the pose differentiation or the odometry.
    if(!input.imu.empty())
    {
        estimate.angular = input.imu.back().rotationalVelocity[IMU_YAW_INDEX];
    }
    
    previousVelocityEstimate_ = estimate;
    
    return estimate;
}


void VelocityEstimator::updatePoseWindow(const pose_distribution_t& newPose)
{
    poseWindow_.push_back(newPose);
    
    // Erase all old poses until the time interval of the pose window is within the desired size
    while((poseWindow_.back().timestamp - poseWindow_.front().timestamp) > (params_.slidingWindowSizeMs * 1000))
    {
        poseWindow_.pop_front();
    }
}


velocity_t VelocityEstimator::slidingWindowVelocity(void)
{
    // The sliding window simply averages the linear and angular motion over a small span of time to determine the current
    // velocity of the robot
    float distanceX = 0.0f;
    float distanceY = 0.0f;
    float rotation  = 0.0f;
    float time      = 0.0f;
    
    for(size_t n = 1; n < poseWindow_.size(); ++n)
    {
        Point<float> delta(poseWindow_[n].x - poseWindow_[n-1].x, poseWindow_[n].y - poseWindow_[n-1].y);
        
        delta = rotate(delta, -poseWindow_[n-1].theta);
        
        distanceX  += delta.x; // now this is forward motion
        distanceY  += delta.y; // side deviation
        rotation   += angle_diff(poseWindow_[n].theta, poseWindow_[n-1].theta);
        time       += utils::usec_to_sec(poseWindow_[n].timestamp - poseWindow_[n-1].timestamp);
    }
    
    float distance = copysign(std::sqrt(distanceX*distanceX + distanceY*distanceY), distanceX);
    
    velocity_t velocity(0.0f, 0.0f);
    velocity.timestamp = poseWindow_.back().timestamp;
    
    if(time > 0.0f)
    {
        velocity.linear  = distance / time;
        velocity.angular = rotation /time;
    }
    
    return velocity;
}


velocity_t VelocityEstimator::odometryVelocity(const std::vector<odometry_t>& odometry)
{
    if(!initialized_)
    {
        lastOdometry_ = odometry.front();
        initialized_  = true;
    }
    
    // The velocity within the last update window is calculated and then filtered with the previous velocity,
    // based on an estimated parameter, to determine the current robot velocity.
    double totalTranslation = 0.0;
    double totalRotation    = 0.0;
    double totalTime        = utils::usec_to_sec(odometry.back().timestamp - lastOdometry_.timestamp);
    
    for(const auto& odom : odometry)
    {
        totalTranslation += odom.translation;
        totalRotation    += odom.rotation;
    }
    
    lastOdometry_ = odometry.back();
    
    velocity_t velocity(0.0f, 0.0f);
    velocity.timestamp = odometry.back().timestamp;
    
    if(totalTime > 0.0)
    {
        double linearVelocity  = totalTranslation / totalTime;
        
        velocity.angular = totalRotation / totalTime;
        velocity.linear  = previousVelocityEstimate_.linear + params_.odometryFilterGain*(linearVelocity - previousVelocityEstimate_.linear);
    }
    
    return velocity;
}

} // namespace robot
} // namespace vulcan
