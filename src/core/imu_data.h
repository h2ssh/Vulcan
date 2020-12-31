/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file
 * \author   Collin Johnson
 *
 * Declaration of imu_data_t, which is produced by all IMU sensors.
 *
 * Declaration of support functions for IMU data:
 *
 *   - interpolate_imu_data
 */

#ifndef CORE_IMU_DATA_H
#define CORE_IMU_DATA_H

#include <cstdint>
#include <utility>

namespace vulcan
{

// Constants defining the position of the various values in the
// *_imu_data_t arrays
const int IMU_X_INDEX = 0;
const int IMU_Y_INDEX = 1;
const int IMU_Z_INDEX = 2;

const int IMU_YAW_INDEX = 0;
const int IMU_PITCH_INDEX = 1;
const int IMU_ROLL_INDEX = 2;


/**
 * imu_data_t contains the data read from a three-axis IMU like the 3DM-GX2
 * that is currently being used by Vulcan.
 *
 * The measurements made by the IMU are: acceleration on all three axes, rotational
 * velocity around all three axes, and the global roll, pitch, yaw of the sensor.
 *
 * IMUs offer other values, like magnetometer readings, but as of now, they are not
 * integrated into our system.
 */
struct imu_data_t
{
    int64_t timestamp;   ///< Time at which the IMU data was read (microseconds)

    int32_t sequenceNumber;   ///< Monotonically increasing number that identifies the particular measurement
    int32_t timeDelta;   ///< Time difference between this reading and last reading in IMU data sequence (microseconds)

    float acceleration[3];         ///< Measured acceleration values (x, y, z)
    float rotationalVelocity[3];   ///< Measured rotational velocities (deltaYaw, deltaPitch, deltaRoll)
    float orientation[3];          ///< Estimate of the global orientation of the robot (yaw, pitch, roll)

    float gravityMagnitude;   ///< Magnitude of the total acceleration due to gravity on the cumulative measured
                              ///< acceleration

    imu_data_t(void) : timestamp(0), sequenceNumber(0), timeDelta(0) { }
};

/**
 * interpolate_imu_data creates a new interpolated IMU measurement between two consecutive IMU measurements. The
 * interpolated measurement uses linear interpolation between the before and after measurements.
 *
 * IMU data contains a sequence number and time delta relative to the previous measurement. As such, the interpolation
 * must create a new measurement and change the measurement coming immediately after the interpolated value. In
 * particular, the sequence number and timeDelta of the after measurement will be adjusted to account for the new entry
 * in the middle of the sequence. Thus, after interpolation, the after measurement must be assigned to the new value.
 * All subsequent measurements will also need to have their sequence numbers changed to reflect the change in sequence
 * numbers caused by the interpolation.
 *
 * \pre  before.timestamp <= time <= after.timestamp
 * \pre  before.timestamp < after.timestamp
 * \param    before          IMU measurement before the desired time
 * \param    after           IMU measurement after the desired time
 * \param    time            Time at which to create the interpolated value
 * \return   A pair of new imu_data_t. first = the interpolated measurement. second = the updated after measurement.
 */
std::pair<imu_data_t, imu_data_t> interpolate_imu_data(const imu_data_t& before, const imu_data_t& after, int64_t time);

}   // namespace vulcan

#endif   // CORE_IMU_DATA_H
