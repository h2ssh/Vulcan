/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pose_estimator.cpp
 * \author   Collin Johnson
 *
 * Implementation of PoseEstimator.
 */

#include "robot/state/pose_estimator.h"
#include "core/vector.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <cassert>
#include <iostream>

// #define DEBUG_INPUT
// #define DEBUG_FILTER

namespace vulcan
{
namespace robot
{

pose_estimator_params_t::pose_estimator_params_t(const utils::ConfigFile& config) : odomParams(config)
{
}


PoseEstimator::PoseEstimator(const pose_estimator_params_t& params)
: params_(params)
, odometryModel_(params.odomParams)
, haveInitialOdometry_(false)
, haveInitialFilteredPose_(false)
{
}


pose_distribution_t PoseEstimator::updateEstimate(const motion_state_input_t& input)
{
    // Odometry data is required
    assert(!input.odometry.empty());

    // Update the stored odometry data
    initializeEstimationIfNeeded(input.odometry.front());

    // Predict new motion from measurements
    addNewOdometryMeasurements(input);
    predictPosesFromOdometry();

    // Process all localized poses for which odometry data exists
    addNewPoseEstimates(input);
    adjustPosesUsingLocalizedPoses();

    if (!predictedPoses_.empty()) {
        return predictedPoses_.back();
    }

    return pose_distribution_t();
}


void PoseEstimator::initializeEstimationIfNeeded(const odometry_t& initialOdom)
{
    if (!haveInitialOdometry_) {
        predictionInput_.push_front(initialOdom);
        haveInitialOdometry_ = true;

        if (predictedPoses_.empty()) {
            pose_distribution_t initialPose;
            initialPose.timestamp = initialOdom.timestamp;
            predictedPoses_.push_front(initialPose);
        }
    }
}


void PoseEstimator::addNewOdometryMeasurements(const motion_state_input_t& measurements)
{
    // Only odometry data is used for estimating the pose
    for (auto& odom : measurements.odometry) {
        // Only odometry data more recent than our previous prediction is useful, as we don't use info from the past to
        // influence future estimates.
        if (predictionInput_.empty() || (odom.timestamp > predictionInput_.back().timestamp)) {
            predictionInput_.push_back(odom);
        } else if (odom.timestamp < predictionInput_.back().timestamp) {
            std::cout
              << "WARNING! PoseEstimator: Odometry must come after previous prediction. Odom time:" << odom.timestamp
              << " Prediction time:" << predictionInput_.back().timestamp
              << " Diff:" << (odom.timestamp - predictionInput_.back().timestamp)
              << " Ignoring update because data is stale. Just using previous measurements as they are more recent.\n";
        }
    }
}


void PoseEstimator::predictPosesFromOdometry(void)
{
    // No work to do if all the calculations have been performed
    if (predictionInput_.empty() || (predictionInput_.size() == predictedPoses_.size())) {
        return;
    }

    assert(!predictedPoses_.empty());
    // Know that predictedPoses_.size() > 0
    // Safe to subtract 1 because we know predictionInput isn't empty
    for (std::size_t n = predictedPoses_.size(); n < predictionInput_.size(); ++n) {
        predictedPoses_.push_back(
          predictNextPose(predictedPoses_[n - 1], predictionInput_[n - 1], predictionInput_[n]));
    }
}


pose_distribution_t PoseEstimator::predictNextPose(const pose_distribution_t& previous,
                                                   const odometry_t& previousOdom,
                                                   const odometry_t& nextOdom)
{
    motion_model_data_t prevData;
    motion_model_data_t nextData;

    prevData.odometry = &previousOdom;
    nextData.odometry = &nextOdom;

    auto distribution = odometryModel_.predictPose(previous, prevData, nextData);
    distribution.timestamp = nextOdom.timestamp;
    return distribution;
}


void PoseEstimator::addNewPoseEstimates(const motion_state_input_t& estimates)
{
    // Store all pose estimates that are more recent than any previous estimates
    for (auto& pose : estimates.localizedPoses) {
        if (localizedPoses_.empty() || (pose.timestamp > localizedPoses_.back().timestamp)) {
            localizedPoses_.push_back(pose);
        }
    }
}


void PoseEstimator::adjustPosesUsingLocalizedPoses(void)
{
    // Keep adjusting estimates until all localized poses are incorporated, or there is no odometry data left for future
    // pose estimates to be made.
    while (!localizedPoses_.empty() && !predictionInput_.empty()) {
        // To incorporate a localized pose, we need to interpolate odometry to get the exact estimate of the amount of
        // motion predicted by odometry. If there is no data before the localized pose, then it can't be used for
        // corrections, so it can be discarded
        if (localizedPoses_.front().timestamp < predictionInput_.front().timestamp) {
            std::cout << "No data before:" << localizedPoses_.front().timestamp
                      << " Earliest:" << predictionInput_.front().timestamp
                      << " Delta: " << (localizedPoses_.front().timestamp - predictionInput_.front().timestamp) << '\n';
            localizedPoses_.pop_front();
        }
        // Otherwise, attempt to incorporate the localized pose into the current estimate of robot pose
        else if (incorporateLocalizedPose(localizedPoses_.front())) {
            localizedPoses_.pop_front();
        } else {
            break;
        }
    }
}


bool PoseEstimator::incorporateLocalizedPose(const pose_distribution_t& localizedPose)
{
    // Can't predict if there are fewer than two predictions
    if (predictionInput_.size() < 2) {
        return false;
    }

    // observationIndex is the index *after* localizedPose.timestamp. It is the first measurement
    // taken after the localization was performed
    std::size_t observationIndex = findIndexAfterTime(localizedPose.timestamp);

    // Didn't find an interval in which the localizedPose fits with the current prediction data
    if (observationIndex == predictionInput_.size()) {
        std::cout << "No odometry after the localized pose. Time:" << localizedPose.timestamp
                  << " Input:" << predictionInput_.back().timestamp << '\n';
        return false;
    }

    // Create an intermediate odometry measurement that corresponds to the time the localized pose was calculated
    // so the predicted pose for the Kalman filter is a better fit (probably doesn't matter too much)
    auto odometryAtLocalizedTime = interpolate_odometry(predictionInput_[observationIndex - 1],
                                                        predictionInput_[observationIndex],
                                                        localizedPose.timestamp);

    bool hasMapChanged = false;
    if (haveInitialFilteredPose_) {
        // Assume the map has changed if there is a sudden large jump in localization, and detect that event
        hasMapChanged = (std::abs(predictedPoses_[observationIndex - 1].x - localizedPose.x) > 2.0)
          || (std::abs(predictedPoses_[observationIndex - 1].y - localizedPose.y) > 2.0)
          || (std::abs(predictedPoses_[observationIndex - 1].theta - localizedPose.theta) > 1.0);
    }

    auto filteredPose = localizedPose;

    if (haveInitialFilteredPose_ && !hasMapChanged) {
        // Apply a linear Kalman filter update, since the prediction and measurement have the same coordinates
        auto filterPrediction = predictNextPose(predictedPoses_[observationIndex - 1],
                                                predictionInput_[observationIndex - 1],
                                                odometryAtLocalizedTime);
        filteredPose = calculateFilteredPose(filterPrediction, localizedPose);
    } else {
        // In case of no initial filtered pose or a new map, just use the localization result.
        haveInitialFilteredPose_ = true;
    }

    // Erase all input up to the observation output because it has been incorporated into the filteredPose
    predictionInput_.erase(predictionInput_.begin(), predictionInput_.begin() + observationIndex);

    // Clear out the predictions and recalculate them using the new filteredPose as the basis for the future predictions
    predictedPoses_.clear();

    assert(predictionInput_.empty() || (odometryAtLocalizedTime.timestamp <= predictionInput_.front().timestamp));
    predictionInput_.push_front(odometryAtLocalizedTime);
    predictedPoses_.push_front(filteredPose);

    predictPosesFromOdometry();

    assert(predictionInput_.size() == predictedPoses_.size());

    return true;
}


std::size_t PoseEstimator::findIndexAfterTime(int64_t observationTime)
{
    std::size_t observationIndex = predictionInput_.size();
    for (std::size_t n = 1; n < predictionInput_.size(); ++n) {
        if (predictionInput_[n - 1].timestamp <= observationTime && predictionInput_[n].timestamp >= observationTime) {
            observationIndex = n;
            break;
        }
    }

#ifdef DEBUG_INPUT
    if (observationIndex == predictionInput_.size()) {
        std::cerr << "WARNING::MotionStateEstimator: Failed to find input interval for observation: " << observationTime
                  << '\n';
    }
#endif

    return observationIndex;
}


pose_distribution_t PoseEstimator::calculateFilteredPose(const pose_distribution_t& predictedPose,
                                                         const pose_distribution_t& localizedPose)
{
    // Do Kalman filter update -- real simple because measurement is exactly same units as prediction
    Matrix innovation = predictedPose.uncertainty.getCovariance()
      * arma::inv(predictedPose.uncertainty.getCovariance() + localizedPose.uncertainty.getCovariance());
    Vector prediction = {predictedPose.x, predictedPose.y, predictedPose.theta};
    Vector measurement = {localizedPose.x, localizedPose.y, localizedPose.theta};
    Vector error = measurement - prediction;

    // Calculate the angle error separately to ensure the wraparound is handled correctly.
    error(2) = angle_diff(localizedPose.theta, predictedPose.theta);

    Vector filteredMean = prediction + innovation * error;

    filteredMean(2) = wrap_to_pi(filteredMean(2));

    pose_distribution_t filteredPose;
    filteredPose.x = filteredMean(0);
    filteredPose.y = filteredMean(1);
    filteredPose.theta = filteredMean(2);
    filteredPose.uncertainty.setDistributionStatistics(filteredMean,
                                                       (arma::eye<Matrix>(3, 3) - innovation)
                                                         * predictedPose.uncertainty.getCovariance());

    filteredPose.timestamp = localizedPose.timestamp;

#ifdef DEBUG_FILTER
    std::cout << "DEBUG::PoseEstimator: Kalman filter results:\n"
              << "Prediction:\n"
              << predictedPose.uncertainty.getMean() << predictedPose.uncertainty.getCovariance() << "Observation:\n"
              << localizedPose.uncertainty.getMean() << localizedPose.uncertainty.getCovariance() << "Filtered:\n"
              << filteredPose.uncertainty.getMean() << filteredPose.uncertainty.getCovariance() << '\n';
#endif   // DEBUG_FILTER

    return filteredPose;
}

}   // namespace robot
}   // namespace vulcan
