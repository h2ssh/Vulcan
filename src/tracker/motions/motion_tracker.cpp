/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     filter.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of tracking_filter_params_t and apply_kalman_filter_update.
*/

#include "tracker/motions/motion_tracker.h"
#include "tracker/laser_object.h"
#include "math/derivatives.h"
#include "utils/config_file.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace tracker
{

const std::string kFilterHeading("LaserTrackingFilterParameters");
const std::string kQSlowKey     ("Q_slow");
const std::string kQFastKey     ("Q_fast");
const std::string kAccUncertaintyKey("acceleration_uncertainty");
const std::string kVelocityWindowKey("velocity_window_duration_ms");
const std::string kSaveMeasurementsKey("should_save_measurements");


void predict_next_state(const MultivariateGaussian& state,
                        MultivariateGaussian&       predicted,
                        float                             deltaTime,
                        const Matrix&               processUncertainty,
                        bool                              isUsingAcceleration);
void innovation(const MultivariateGaussian& predicted,
                const MultivariateGaussian& measurement,
                Vector&                     innovation,
                Matrix&                     kalmanGain);
void update_state(const MultivariateGaussian& predicted,
                  const Vector&               innovation,
                  const Matrix&               kalmanGain,
                  MultivariateGaussian&       updated);

/////////////////////////// tracking_filter_params_t /////////////////////////////////

tracking_filter_params_t::tracking_filter_params_t(void)
: saveMeasurementData(false)
{
}


tracking_filter_params_t::tracking_filter_params_t(const utils::ConfigFile& config)
: Qslow(config.getValueAsMatrix(kFilterHeading, kQSlowKey))
, Qfast(config.getValueAsMatrix(kFilterHeading, kQFastKey))
, accelerationUncertainty(config.getValueAsFloat(kFilterHeading, kAccUncertaintyKey))
, velocityWindowDurationMs(config.getValueAsInt32(kFilterHeading, kVelocityWindowKey))
, saveMeasurementData(config.getValueAsBool(kFilterHeading, kSaveMeasurementsKey))
{
    assert(Qslow.n_cols == stateSize && Qslow.n_rows == stateSize);
    assert(Qfast.n_cols == stateSize && Qfast.n_rows == stateSize);
    assert(accelerationUncertainty >= 0.0f);
}

/////////////////////////// LinearMotionTracker /////////////////////////////////

int64_t LinearMotionTracker::kId_ = 0;


LinearMotionTracker::LinearMotionTracker(void)
{
}


LinearMotionTracker::LinearMotionTracker(int64_t  timestamp,
                                         Position initialPosition,
                                         const tracking_filter_params_t& params)
: timestamp_(timestamp)
, slowState_(stateSize)
, fastState_(stateSize)
, positions_(params.velocityWindowDurationMs * 1000ll)
, velocities_(params.velocityWindowDurationMs * 1000ll)
, params_(params)
{
    Vector initialState(stateSize);
    Matrix initialUncertainty(stateSize, stateSize);

    initialState.zeros();
    initialUncertainty.zeros();

    initialState(xIndex) = initialPosition.x;
    initialState(yIndex) = initialPosition.y;

    // position uncertainty is inherited from direct observation
    initialUncertainty(xIndex, xIndex) = initialUncertainty(yIndex, yIndex) = 0.1;

    // velocity is not directly observable until the next observation. States are set to zero with high initial uncertainty
    // so that it can quickly corrected during the filtering process.
    initialUncertainty(velXIndex, velXIndex) = initialUncertainty(velYIndex, velYIndex) = 1000;
    initialUncertainty(accelXIndex, accelXIndex) = initialUncertainty(accelYIndex, accelYIndex) = 1000;

    slowState_.setDistributionStatistics(initialState, initialUncertainty);
    fastState_.setDistributionStatistics(initialState, initialUncertainty);

    if(params.saveMeasurementData)
    {
        int id = kId_++;
        std::ostringstream filename;
        filename << "tracker_meas_" << id << ".dat";
        measurementsOut_.open(filename.str());
    }
}


LinearMotionTracker::LinearMotionTracker(const LinearMotionTracker& rhs)
: timestamp_(rhs.timestamp_)
, slowState_(rhs.slowState_)
, fastState_(rhs.fastState_)
, positions_(rhs.positions_)
, velocities_(rhs.velocities_)
, params_(rhs.params_)
{
    if(params_.saveMeasurementData)
    {
        int id = kId_++;
        std::ostringstream filename;
        filename << "tracker_meas_" << id << ".dat";
        measurementsOut_.open(filename.str());
    }
}


object_motion_state_t LinearMotionTracker::slowState(void) const
{
    return object_motion_state_t(slowState_[xIndex],
        slowState_[yIndex],
        slowState_[velXIndex],
        slowState_[velYIndex],
        slowState_[accelXIndex],
        slowState_[accelYIndex]
    );
}


object_motion_state_t LinearMotionTracker::fastState(void) const
{
    return object_motion_state_t(fastState_[xIndex],
        fastState_[yIndex],
        fastState_[velXIndex],
        fastState_[velYIndex],
        fastState_[accelXIndex],
        fastState_[accelYIndex]
    );
}


void LinearMotionTracker::update(const LaserObject& object)
{
    addMeasurement(object);

    double deltaTime = utils::usec_to_sec(object.timestamp() - timestamp_);

    if(deltaTime > 0.0)
    {
        MultivariateGaussian measurement = motionMeasurement();
        MultivariateGaussian predictedState(stateSize);
        Vector innov(stateSize);
        Matrix uncertainty(stateSize, stateSize);

        predict_next_state(slowState_, predictedState, deltaTime, params_.Qslow, true);
        innovation(predictedState, measurement, innov, uncertainty);
        update_state(predictedState, innov, uncertainty, slowState_);

        predict_next_state(fastState_, predictedState, deltaTime, params_.Qfast, true);
        innovation(predictedState, measurement, innov, uncertainty);
        update_state(predictedState, innov, uncertainty, fastState_);

        detectStopsAndTurns();

        saveMeasurementsIfRequested(object.timestamp(), measurement);
    }

    timestamp_ = object.timestamp();
}


void LinearMotionTracker::addMeasurement(const LaserObject& object)
{
    positions_.push(uncertain_measurement_t(object.timestamp(), object.centerWithUncertainty()));

    // If there are enough position measurements, then take the derivative to get the measured velocity estimate
    if(positions_.isFull())
    {
        auto velocityEstimate = uncertainSeqDeriv(positions_);
        velocities_.push(uncertain_measurement_t(positions_.timestamp(), velocityEstimate));
    }
}


MultivariateGaussian LinearMotionTracker::motionMeasurement(void) const
{
    // Create the motion measurement using the measured position and the derived velocity and acceleration measurements
    Vector motionMean(stateSize);
    Matrix motionCov(stateSize, stateSize);

    motionMean.zeros();
    motionCov.zeros();

    // The position always exists
    motionMean.subvec(xIndex, yIndex) = positions_.back().measurement.getMean();
    motionCov.submat(xIndex, xIndex, yIndex, yIndex) = positions_.back().measurement.getCovariance();

    // A velocity doesn't always exist because it needs two measurements before it can be estimated
    if(!velocities_.isFull())
    {
        // If there aren't velocities set the measurement covariance very high
        motionCov.submat(velXIndex, velXIndex, velYIndex, velYIndex) = arma::eye(2, 2) * 1000;
    }
    else
    {
        motionMean.subvec(velXIndex, velYIndex) = velocities_.back().measurement.getMean();
        motionCov.submat(velXIndex, velXIndex, velYIndex, velYIndex) = velocities_.back().measurement.getCovariance();
    }

    // An acceleration estimate doesn't always exist if there aren't enough velocity measurements
    if(!velocities_.isFull())
    {
        motionCov.submat(accelXIndex, accelXIndex, accelYIndex, accelYIndex) = arma::eye(2, 2) * 1000;
    }
    else
    {
        // Take the derivative of the stored velocities to get the acceleration estimate
        auto measuredAccel = uncertainSeqDeriv(velocities_);
        motionMean.subvec(accelXIndex, accelYIndex) = measuredAccel.getMean();
        motionCov.submat(accelXIndex, accelXIndex, accelYIndex, accelYIndex) = measuredAccel.getCovariance();
    }

    return MultivariateGaussian(motionMean, motionCov);
}


MultivariateGaussian LinearMotionTracker::uncertainSeqDeriv(const utils::FixedDurationBuffer<uncertain_measurement_t>& data) const
{
    // Combine all uncertain measurements in information form, then convert back to moments parameterization
    auto dimensions = data.front().measurement.getMean().n_rows;
    Vector infoVector(dimensions);
    Matrix infoMatrix(dimensions, dimensions);

    infoVector.zeros();
    infoMatrix.zeros();

    for(std::size_t n = 1; n < data.size(); ++n)
    {
        // Can only take derivatives if there is a time difference
        if(data[n-1].timestamp == data[n].timestamp)
        {
            continue;
        }

        auto deriv = math::uncertain_time_derivative(data[n-1].measurement,
                                                     data[n].measurement,
                                                     data[n].timestamp - data[n-1].timestamp);

        Matrix derivInfo = arma::inv(deriv.getCovariance());
        Vector derivVec = derivInfo * deriv.getMean();

        infoVector += derivVec;
        infoMatrix += derivInfo;
    }

    return MultivariateGaussian(arma::inv(infoMatrix) * infoVector, arma::inv(infoMatrix));
}


void LinearMotionTracker::detectStopsAndTurns(void)
{
    // state from the fast filter
//     Vector fastMean = fastState_.getMean();
//
//     // detect stop with fast filter
//     double objectSpeedInFastFilter = sqrt(fastMean[velXIndex]*fastMean[velXIndex] + fastMean[velYIndex]*fastMean[velYIndex]);
//     if(objectSpeedInFastFilter < 0.2)
//     {
//         // override slow state if stopped
//         slowState_.setDistributionStatistics(fastMean, fastState_.getCovariance());
//     }

    // TODO: do something with turning here


//     // thresholding velocities. Are these good numbers? Maybe.
//     double kSpeedIsVerySlowThreshold = 0.1; // (m/s)
//     double kSpeedIsNotSlowThreshold  = 0.3; // (m/s)
//     double kSpeedIsSimilarThreshold  = 0.2; // (m/s)
//     double kLargeChangeInDirection   = 0.1; // (rad)
//
//     // positionState contains predictions from fast filter, and the velocity contains prediction from slow filter.
//     // for stability controller follows the slow mean, but when sudden stopping motion is detected we need to
//     // update the slow filter output immediately to avoid collision.
//     Vector fastMean = fastState_.getMean();
//     Vector slowMean = slowState_.getMean();
//
//     double objectSpeedInFastFilter = sqrt(fastMean[velXIndex]*fastMean[velXIndex] + fastMean[velYIndex]*fastMean[velYIndex]);
//     double objectSpeedInSlowFilter = sqrt(slowMean[velXIndex]*slowMean[velXIndex] + slowMean[velYIndex]*slowMean[velYIndex]);
//
//     // detect sudden *drop* in speed
//     if(objectSpeedInSlowFilter > kSpeedIsVerySlowThreshold)
//     {
//         // check the drop of more than stateSize0%
//         if(objectSpeedInFastFilter < 0.6*objectSpeedInSlowFilter)
//         {
//             // force update the slow filter states to match the fast filter
//             slowState_.setDistributionStatistics(fastMean, fastState_.getCovariance());
//         }
//     }
//
//     // detect objects suddenly switching *direction* of motion, i.e. turning around a corner.
//     // only consider objects that are moving at some speed
//     if(objectSpeedInSlowFilter > kSpeedIsNotSlowThreshold)
//     {
//         double objectSpeedDifference = fabs(objectSpeedInFastFilter - objectSpeedInSlowFilter);
//
//         if(objectSpeedDifference < kSpeedIsSimilarThreshold)
//         {
//             double objectHeadingInFastFilter = atan2(fastMean[velYIndex], fastMean[velXIndex]);
//             double objectHeadingInSlowFilter = atan2(slowMean[velYIndex], slowMean[velXIndex]);
//             double objectHeadingDifference = angle_diff_abs(objectHeadingInFastFilter, objectHeadingInSlowFilter);
//
//             // if the heading error is largem switch to the fast filter.
//             if(objectHeadingDifference > kLargeChangeInDirection)
//             {
//                 slowState_.setDistributionStatistics(fastMean, fastState_.getCovariance());
//             }
//         }
//     }
}


void LinearMotionTracker::saveMeasurementsIfRequested(int64_t timestamp, const MultivariateGaussian& measurement)
{
    // If not requested, then just exit out
    if(!params_.saveMeasurementData)
    {
        return;
    }

    // If there aren't any measurements yet, then don't save things
    if(!velocities_.isFull())
    {
        return;
    }

    measurementsOut_ << timestamp << ' '
        << measurement[velXIndex] << ' ' << measurement(velXIndex,velXIndex) << ' '
        << measurement[velYIndex] << ' ' << measurement(velYIndex,velYIndex) << ' '
        << std::sqrt((measurement[velXIndex] * measurement[velXIndex]) + (measurement[velYIndex] * measurement[velYIndex])) << ' '
        << measurement[accelXIndex] << ' ' << measurement(accelXIndex, accelXIndex) << ' '
        << measurement[accelYIndex] << ' ' << measurement(accelYIndex, accelYIndex) << ' '
        << std::sqrt((measurement[accelXIndex] * measurement[accelXIndex]) + (measurement[accelYIndex] * measurement[accelYIndex]))
        << std::endl;
}


void predict_next_state(const MultivariateGaussian& state,
                        MultivariateGaussian&       predicted,
                        float                             deltaTime,
                        const Matrix&               processUncertainty,
                        bool                              isUsingAcceleration)
{
    /*
    * The process matrix estimates the predicts the state based on the previous state.
    *
    *   - The position is changed by the velocity and acceleration estimates
    *   - The velocity is changed by the accleration estimate.
    *   - The acceleration is assumed constants.
    */

    Matrix A(stateSize, stateSize);
    A.zeros();
    A(xIndex, xIndex) = 1;
    A(xIndex, velXIndex) = deltaTime;
    A(yIndex, yIndex) = 1;
    A(yIndex, velYIndex) = deltaTime;
    A(velXIndex, velXIndex) = 1;
    A(velYIndex, velYIndex) = 1;
    A(accelXIndex, accelXIndex) = 1;
    A(accelYIndex, accelYIndex) = 1;

    if(isUsingAcceleration)
    {
        A(xIndex, accelXIndex) = 0.5 * deltaTime * deltaTime;
        A(yIndex, accelYIndex) = 0.5 * deltaTime * deltaTime;
        A(velXIndex, accelXIndex) = deltaTime;
        A(velYIndex, accelYIndex) = deltaTime;
    }

    Vector predictedState = A * state.getMean();
    Matrix predictedSigma = A * state.getCovariance() * trans(A) + processUncertainty;
    predicted.setDistributionStatistics(predictedState, predictedSigma);
}


void innovation(const MultivariateGaussian& predicted,
                const MultivariateGaussian& measurement,
                Vector&                     innovation,
                Matrix&                     kalmanGain)
{
    innovation = measurement.getMean() - predicted.getMean();
    kalmanGain = predicted.getCovariance() * inv(predicted.getCovariance() + measurement.getCovariance());
}


void update_state(const MultivariateGaussian& predicted,
                  const Vector&               innovation,
                  const Matrix&               kalmanGain,
                  MultivariateGaussian&       updated)
{
    Vector updatedMean = predicted.getMean() + (kalmanGain * innovation);
    Matrix updatedCov  = (arma::eye(stateSize, stateSize) - kalmanGain)
        * predicted.getCovariance();

    updated.setDistributionStatistics(updatedMean, updatedCov);
}

} // namespace tracker
} // namespace vulcan
