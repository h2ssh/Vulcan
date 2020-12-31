/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     odometry.h
 * \author   Collin Johnson
 *
 * Declaration of odometry_t, encoder_data_t.
 *
 * Declaration of odometry support functions:
 *
 *   - interpolate_odometry
 */

#ifndef CORE_ODOMETRY_H
#define CORE_ODOMETRY_H

#include <cstdint>

namespace vulcan
{

/**
 * encoder_data_t contains information read from wheel encoders on the robot.
 */
struct encoder_data_t
{
    int64_t timestamp;
    int32_t id;

    double deltaLeftWheel;            ///< Measured motion of the left wheel in meters
    double deltaRightWheel;           ///< Measured motion of the right wheel in meters
    double wheelbase;                 ///< Wheelbase of the robot
    double leftWheelCircumference;    ///< In meters
    double rightWheelCircumference;   ///< In meters
    int32_t leftTicksPerRevolution;
    int32_t rightTicksPerRevolution;

    double leftRPM;    ///< Measured RPM on the left wheel
    double rightRPM;   ///< Measured RPM on the right wheel

    int64_t leftTicksTotal;    ///< Cumulative ticks counted on the left wheel
    int64_t rightTicksTotal;   ///< Cumulative ticks counted on the right wheel

    int64_t leftIndexPulseTotal;    ///< Total number of index pulses counted on the left wheel
    int64_t rightIndexPulseTotal;   ///< Total number of index pulses counted on the right wheel

    encoder_data_t(void)
    : timestamp(0)
    , id(0)
    , deltaLeftWheel(0.0)
    , deltaRightWheel(0.0)
    , leftRPM(0.0)
    , rightRPM(0.0)
    , leftTicksTotal(0)
    , rightTicksTotal(0)
    , leftIndexPulseTotal(0)
    , rightIndexPulseTotal(0)
    {
    }
};

/**
 * odometry_t contains the information about a single odometry update. The odometry is
 * defined by the (x,y,theta) of the robot as determined by dead-reckoning. To determine
 * motion between two odometry readings, simply take the difference of the two.
 */
struct odometry_t
{
    int64_t timestamp;   ///< Time at which the measurement was made
    int32_t id;          ///< Monotonically increasing id so missing odometry can be easily identified

    double x;       ///< Dead-reckoning x-position of the robot
    double y;       ///< Dead-reckoning y-position of the robot
    double theta;   ///< Dead-reckoning orientation of the robot

    double translation;   ///< Distance traveled between last measurement and this measurement
    double rotation;      ///< Amount of rotation between last measurement and this measurement

    odometry_t(void) : timestamp(0), x(0), y(0), theta(0), translation(0), rotation(0) { }
};

/**
 * interpolate_odometry linearly interpolates between two odometry measurements to create a new artificial measurement.
 * The interpolation assumes a constant velocity of the robot between the two measurements.
 *
 * \pre before.timestamp <= time <= after.timestamp
 * \param    before          Odometry measurement taken before the desired time
 * \param    after           Odometry measurement taken after the desired time
 * \param    time            Time at which to create the interpolated measurement
 * \return   Interpolated odometry measurement.
 */
odometry_t interpolate_odometry(const odometry_t& before, const odometry_t& after, int64_t time);


template <class Archive>
void serialize(Archive& ar, odometry_t& odom, const unsigned int version)
{
    ar& odom.timestamp;
    ar& odom.id;
    ar& odom.x;
    ar& odom.y;
    ar& odom.theta;
    ar& odom.translation;
    ar& odom.rotation;
}

}   // namespace vulcan

#endif   // CORE_ODOMETRY_H
