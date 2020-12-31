/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     filter.h
 * \author   Collin Johnson
 *
 * Declaration of tracking_filter_params_t and apply_kalman_filter_update.
 */

#ifndef LASER_TRACKER_FILTER_H
#define LASER_TRACKER_FILTER_H

#include "core/multivariate_gaussian.h"
#include "tracker/object_state.h"
#include "tracker/types.h"
#include "utils/fixed_duration_buffer.h"
#include <cstdint>
#include <fstream>

namespace vulcan
{

class MultivariateGaussian;

namespace utils
{
class ConfigFile;
}

namespace tracker
{

class LaserObject;

/**
 * tracking_filter_params_t defines parameters for the filter used to track a steady object's state.
 *
 *   [TrackingFilterParameters]
 *   Q_slow = 6x6 matrix specifying how process noise grows when estimating slow-changing state
 *   Q_fast = 6x6 matrix specifying how process noise grows when estimating fast-changing state
 *   acceleration_uncertainty = increase in uncertainty of measurement due to measured acceleration of the object
 *   velocity_window_duration_ms =
 *   should_save_measurements = flag indicating if the measurement data needs to be saved for debugging use
 */
struct tracking_filter_params_t
{
    Matrix Qslow;
    Matrix Qfast;
    float accelerationUncertainty;
    int32_t velocityWindowDurationMs;
    bool saveMeasurementData;

    tracking_filter_params_t(void);
    tracking_filter_params_t(const utils::ConfigFile& config);
};


enum MotionStateIndex
{
    xIndex,
    yIndex,
    velXIndex,
    velYIndex,
    accelXIndex,
    accelYIndex,
    stateSize,
};


/**
 * LinearMotionTracker tracks the state of an object based on the measurement contained
 * in a laser object. The update applies a basic linear Kalman filter to estimate (x, y, v_x, v_y, a_x, a_y)
 * for the tracked object incorporating the new measurement.
 *
 * The tracked object is (x, y, v_x, v_y, accel_x, accel_y). The laser object is just (x, y), so the velocity and
 * accelerations are derived based on the time difference between consecutive measurements. The Kalman filter accepts
 * two parameters defined in tracking_filter_params_t.
 *
 * When data is being saved, the format is:
 *
 *   timestamp v_x var_v_x v_y var_v_y v_mag a_x var_a_x a_y var_a_y a_mag
 *      1      2    3      4     5      6    7    8      9     10     11
 */
class LinearMotionTracker
{
public:
    /**
     * Default constructor for LinearMotionTracker.
     */
    LinearMotionTracker(void);

    /**
     * Constructor for LinearMotionTracker.
     *
     * \param    timestamp           Initial timestamp for the tracker
     * \param    initialPosition     Initial position of the object
     * \param    params              Parameters for the internal Kalman filter
     */
    LinearMotionTracker(int64_t timestamp, Position initialPosition, const tracking_filter_params_t& params);

    /**
     * Copy constructor for LinearMotionTracker.
     */
    LinearMotionTracker(const LinearMotionTracker& rhs);

    /**
     * timestamp retrieves the time at which the last update was made to the object.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * slowState retrieves the state estimate using low process noise.
     */
    object_motion_state_t slowState(void) const;

    /**
     * fastState retrieves the state estimate using high process noise.
     */
    object_motion_state_t fastState(void) const;

    /**
     * slowStateWithUncertainty retrieves the full slow state estimate being maintained by the tracker.
     */
    MultivariateGaussian slowStateWithUncertainty(void) const { return slowState_; }

    /**
     * fastStateWithUncertainty retrieves the full fast state estimate being maintained by the tracker.
     */
    MultivariateGaussian fastStateWithUncertainty(void) const { return fastState_; }

    /**
     * update updates the tracked estimate of the position and velocity of the object. No update is performed if
     * object.timestap() <= timestamp(). The state is only predicted into the future.
     *
     * \param    object          Measurement of the object's position
     */
    void update(const LaserObject& object);

private:
    struct uncertain_measurement_t
    {
        int64_t timestamp;
        MultivariateGaussian measurement;

        uncertain_measurement_t(int64_t timestamp, MultivariateGaussian measurement)
        : timestamp(timestamp)
        , measurement(measurement)
        {
        }
    };

    static int64_t kId_;

    int64_t timestamp_;
    MultivariateGaussian slowState_;
    MultivariateGaussian fastState_;
    utils::FixedDurationBuffer<uncertain_measurement_t> positions_;
    utils::FixedDurationBuffer<uncertain_measurement_t> velocities_;
    tracking_filter_params_t params_;

    std::ofstream measurementsOut_;

    void addMeasurement(const LaserObject& object);
    MultivariateGaussian motionMeasurement(void) const;
    MultivariateGaussian uncertainSeqDeriv(const utils::FixedDurationBuffer<uncertain_measurement_t>& data) const;

    void detectStopsAndTurns(void);
    void saveMeasurementsIfRequested(int64_t timestamp, const MultivariateGaussian& measurement);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, slowState_, fastState_);
    }
};

}   // namespace tracker
}   // namespace vulcan

#endif   // LASER_TRACKER_FILTER_H
