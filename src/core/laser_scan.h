/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_scan.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of polar_laser_scan_t and cartesian_laser_scan_t types.
 */

#ifndef CORE_LASER_SCAN_H
#define CORE_LASER_SCAN_H

#include "core/point.h"
#include "core/pose.h"
#include "system/message_traits.h"
#include <cereal/types/vector.hpp>
#include <cstdint>

namespace vulcan
{

enum : int32_t
{
    kFrontLaserId,   ///< Id of the front laser on Vulcan
    kBackLaserId,    ///< Id of the back laser on Vulcan
    kNumLasers
};

/**
 * polar_laser_scan_t contains the data from a single scan of a laser rangefinder. The data is provided
 * in its polar form, just like it is measured, because potentially useful information is lost
 * when converting to Cartesian coordinates.
 *
 * The representation of a laser scan is a start angle in robot coordinates, an angular resolution
 * that specifies the change in angle between each scan (- = CCW scan, + = CW scan), and an array
 * of ranges that are the actual measured values.
 *
 * Ranges with a negative value should be ignored. They are ranges indicated by the driver to be not useful.
 * In the case of the wheelchair, these are the laser points known to hit some part of the wheelchair or user.
 */
struct polar_laser_scan_t
{
    int32_t laserId;   ///< Unique id assigned to the source laser --- guaranteed to start at 0 and be sequential for
                       ///< each new laser added

    int64_t timestamp;   ///< Time at which the measurement was taken
    int32_t scanId;      ///< Monotonically increasing ID for the scans to allow multiple modules to sync on laser scans

    float startAngle;          ///< Angle the first range in the scan points, in laser coordinates
    float angularResolution;   ///< Change in angle between each measurement. - = CCW scan, + = CW scan

    uint16_t numRanges;                  ///< Number of range values in the scan
    std::vector<float> ranges;           ///< Measured ranges -- negative values should be ignored as bad readings
    std::vector<uint16_t> intensities;   ///< Intensity values for each of the ranges

    // Parameters of the particular rangefinder
    float maxRange;       ///< Maximum range that can be measured by the laser
    float scanPeriod;     ///< Seconds per rotation of 2pi radians. Equivalent to 60/rpm of laser
    pose_6dof_t offset;   ///< Offset of the rangefinder from the center of the robot coordinate frame

    polar_laser_scan_t(void) : startAngle(0), angularResolution(0), numRanges(0), maxRange(0) { }

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& laserId;
        ar& timestamp;
        ar& scanId;
        ar& startAngle;
        ar& angularResolution;
        ar& numRanges;
        ar& ranges;
        ar& intensities;
        ar& maxRange;
        ar& scanPeriod;
        ar& offset;
    }
};


/**
 * cartesian_laser_scan_t contains data from a single scan of a laser rangefinder. The points are in the same
 * arrangement as when they were originally gathered from the laser in polar coordinates.
 */
struct cartesian_laser_scan_t
{
    int64_t timestamp;

    uint16_t numPoints;
    std::vector<Point<float>> scanPoints;   ///< Measured points
    std::vector<uint16_t> intensities;      ///< Intensity values for each of the ranges

    pose_t offsetFromRobotCenter;

    cartesian_laser_scan_t(void) : timestamp(0), numPoints(0) { }
};

void polar_scan_to_cartesian_scan(const polar_laser_scan_t& polarScan,
                                  cartesian_laser_scan_t& cartesianScan,
                                  bool filterBadValues = false);

void polar_scan_to_cartesian_scan_in_robot_frame(const polar_laser_scan_t& polarScanInRobotFrame,
                                                 cartesian_laser_scan_t& cartesianScanInRobotFrame,
                                                 bool filterBadValues = false);

void polar_scan_to_cartesian_scan_in_global_frame(const polar_laser_scan_t& polarScanInRobotFrame,
                                                  const pose_t& robotPoseInGrlbalFrame,
                                                  cartesian_laser_scan_t& cartesianScanInGlobalFrame,
                                                  bool filterBadValues = false);

// Extra implementations for rendering
void polar_scan_to_cartesian_scan_in_robot_frame(const polar_laser_scan_t& polarScanInLaserFrame,
                                                 const pose_6dof_t& laserPoseInRobotFrame,
                                                 cartesian_laser_scan_t& cartesianScanInLaserFrame,
                                                 bool filterBadValues = false);

void polar_scan_to_cartesian_scan_in_global_frame(const polar_laser_scan_t& polarScanInRobotFrame,
                                                  const pose_6dof_t& laserPoseInRobotFrame,
                                                  const pose_t& robotPoseInGlobalFrame,
                                                  cartesian_laser_scan_t& cartesianScanInGlobalFrame,
                                                  bool filterBadValues = false);

// Operators
/**
 * A laser scan is less than another one if it has a smaller timestamp. This allows laser scans to be easily sorted and
 * searched via time.
 */
inline bool operator<(const polar_laser_scan_t& lhs, const polar_laser_scan_t& rhs)
{
    return lhs.timestamp < rhs.timestamp;
}

inline bool operator>(const polar_laser_scan_t& lhs, const polar_laser_scan_t& rhs)
{
    return lhs.timestamp > rhs.timestamp;
}

namespace system
{

template <>
struct message_traits<polar_laser_scan_t>
{
    static const std::size_t kFrontLaserChannel = 0;
    static const std::size_t kBackLaserChannel = 1;

    using type = old_message_tag;

    enum
    {
        num_channels = 2
    };

    static std::string channelName(std::size_t index)
    {
        assert(index < 2);
        if (index == kFrontLaserChannel) {
            return "SENSOR_LASER_FRONT_6DOF";
        } else   // if(index == kBackLaserChannel)
        {
            return "SENSOR_LASER_BACK_6DOF";
        }
    }
};

}   // namespace system
}   // namespace vulcan

#endif   // CORE_LASER_SCAN_H
