/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_estimator.h
* \author   Collin Johnson
* 
* Definition of PoseEstimator piece of the overall MotionStateEstimator.
*/

#ifndef ROBOT_STATE_POSE_ESTIMATOR_H
#define ROBOT_STATE_POSE_ESTIMATOR_H

#include <robot/state/motion_state_input.h>
#include <core/pose_distribution.h>
#include <robot/model/odometry_models.h>
#include <deque>

namespace vulcan
{
namespace robot
{
    
/**
* pose_estimator_params_t defines the parameters for doing the pose estimation.
*/
struct pose_estimator_params_t
{
    odometry_model_params_t odomParams;
    
    pose_estimator_params_t(const utils::ConfigFile& config);
};

/**
* PoseEstimator
*/
class PoseEstimator
{
public:
    
    /**
    * Constructor for PoseEstimator.
    * 
    * \param    params      Parameters controlling the pose estimation behavior
    */
    PoseEstimator(const pose_estimator_params_t& params);
    
    /**
    * updateEstimate updates the current pose estimate with new data.
    */
    pose_distribution_t updateEstimate(const motion_state_input_t& input);
    
private:
    
    pose_estimator_params_t params_;
    
    // INVARIANT: predictedPoses_[0] is the result of applying the motion model using predictionInput_[0] to filteredPose_
    // INVARIANT: predictedPoses_[n] is the result of applying the motion model using predictionInput_[n] to predictedPoses_[n-1]
    std::deque<pose_distribution_t> predictedPoses_;
    std::deque<odometry_t> predictionInput_;
    std::deque<pose_distribution_t> localizedPoses_;    // Localized poses that need to be incorporated
    
    InverseOdometryModel odometryModel_;

    bool haveInitialOdometry_;
    bool haveInitialFilteredPose_;

    void initializeEstimationIfNeeded(const odometry_t& initialOdom);
    void addNewOdometryMeasurements(const motion_state_input_t& measurements);
    void predictPosesFromOdometry(void);
    pose_distribution_t predictNextPose(const pose_distribution_t& previous, 
                                        const odometry_t& previousOdom,
                                        const odometry_t& nextOdom);
    void addNewPoseEstimates(const motion_state_input_t& estimates);
    void adjustPosesUsingLocalizedPoses(void);
    bool incorporateLocalizedPose(const pose_distribution_t& localizedPose);
    
    std::size_t findIndexAfterTime(int64_t observationTime);
    pose_distribution_t calculateFilteredPose(const pose_distribution_t& predictedPose,
                                              const pose_distribution_t& localizedPose);
};

}
}

#endif // ROBOT_STATE_POSE_ESTIMATOR_H
