/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data.h
 * \author   Collin Johnson
 *
 * Definition of metric_slam_data_t.
 */

#ifndef HSSH_UTILS_METRICAL_DATA_H
#define HSSH_UTILS_METRICAL_DATA_H

#include "core/imu_data.h"
#include "core/laser_scan.h"
#include "core/odometry.h"
#include "core/velocity.h"
#include "laser/laser_scan_lines.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
 * metric_slam_data_t defines the sensor input to the LPM for one update cycle.
 *
 * Update cycles are synchronized with the receipt of laser data, so there will
 * be only one piece of laser data per update. IMU and motor commands are issued
 * at a faster rate than the laser data, though, so multiple pieces of such data
 * will exist in each update cycle.
 *
 * INVARIANTS:
 *   endTime > startTime
 *   laser.timestamp == endTime
 *   imu.front().timestamp == startTime
 *   imu.back().timestamp == endTime
 *   odometry.front().timestamp == startTime
 *   odometry.back().timestamp == endTime
 */
struct metric_slam_data_t
{
    int64_t startTime;   // beginning of the time interval for the sensor data
    int64_t endTime;     // end of the time interval for the sensor data

    velocity_t velocity;   ///< Measured velocity of the robot

    std::vector<imu_data_t> imu;        ///< IMU data gathered between startTime and endTime
    std::vector<odometry_t> odometry;   ///< Odometry data gathered between startTime and endTime

    polar_laser_scan_t laser;              ///< Laser scan to be used for doing the full SLAM update
    laser::laser_scan_lines_t scanLines;   // lines+scan from which they were extracted
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_DATA_H
