/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     carmen_reader.cpp
* \author   Collin Johnson
*
* Definition of CarmenReader.
*/

#include <logging/logplayer/carmen_reader.h>
#include <core/angle_functions.h>
#include <utils/timestamp.h>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

// #define DEBUG_MESSAGES

namespace vulcan
{
namespace logplayer
{

const unsigned int MAX_CARMEN_MESSAGE_LENGTH = 100000;

const std::string OLD_LASER_HEADING("FLASER");
const std::string PARAM_HEADING    ("PARAM");
const std::string ROBOT_LASER_HEADING("ROBOTLASER1");
const std::string COMMENT_HEADING("#");

const std::string PARAM_ROBOT_LENGTH  ("robot_length");
const std::string PARAM_ROBOT_WIDTH   ("robot_width");
const std::string PARAM_LASER_OFFSET  ("robot_frontlaser_offset");
const std::string PARAM_LASER_MAXRANGE("robot_front_laser_max");


CarmenReader::CarmenReader(const data_channels_t& channels)
: LogReader(channels)
, havePreviousPose(false)
, previousLaserTimestamp(0)
, previousAssignedTimestamp(0)
{
}


bool CarmenReader::convertLogToFrames(const std::string& filename)
{
    std::ifstream log(filename);

    if (!log.is_open())
    {
        std::cerr << "ERROR::CarmenReader: Failed to open log " << filename << '\n';
        return false;
    }

    previousPose.timestamp = 0;
    havePreviousPose = false;
    previousLaserTimestamp = 0;
    previousAssignedTimestamp = 0;

    std::string messageType;

    for (std::string message; std::getline(log, message);)
    {
        std::istringstream in(message);
        in >> messageType;

        processMessage(messageTypeFromString(messageType), in);
    }

//     sortFramesByTimestamp();

    return true;
}


CarmenReader::carmen_message_type_t CarmenReader::messageTypeFromString(const std::string& typeString)
{
    if(typeString == OLD_LASER_HEADING)
    {
        return OLD_LASER_MESSAGE;
    }
    else if(typeString == PARAM_HEADING)
    {
        return PARAM_MESSAGE;
    }
    else if(typeString == ROBOT_LASER_HEADING)
    {
        return ROBOT_LASER_MESSAGE;
    }
    else
    {
        return UNKNOWN_MESSAGE;
    }
}


void CarmenReader::processMessage(carmen_message_type_t type, std::istringstream& message)
{
    switch(type)
    {
    case OLD_LASER_MESSAGE:
        processOldLaserMessage(message);
        break;

    case ROBOT_LASER_MESSAGE:
        processLaserMessage(message);
        break;

    case PARAM_MESSAGE:
        readParamMessage(message);
        break;

    // Skip any messages that aren't recognized, as they aren't needed for log playback
    case UNKNOWN_MESSAGE:
    default:
        break;
    }
}


void CarmenReader::processOldLaserMessage(std::istringstream& message)
{
    /*
     * Format of old laser message:
     *
     *  num_rays ray_0 ...ray_N x_scan y_scan theta_scan odom_x odom_y odom_theta timestamp hostname log_time
     */

    data_frame_t frame;

    polar_laser_scan_t& scan = frame.scan;

    message >> scan.numRanges;
    scan.ranges.resize(scan.numRanges);

    std::copy_n(std::istream_iterator<double>(message), scan.numRanges, scan.ranges.begin());

    scan.offset = pose_t(0.0f, 0.0f, 0.0f);
    scan.laserId = 0;

    scan.angularResolution = M_PI / scan.numRanges;
    scan.startAngle = -M_PI / 2.0;
    scan.maxRange = maxLaserRange;
    scan.scanPeriod = 1.0 / 75.0;       // assume laser is a SICK @ 75Hz

    odometry_t& odom = frame.odometry;

    message >> odom.x;
    message >> odom.y;
    message >> odom.theta;

    // skip odom x,y,theta
    std::istream_iterator<double> skipIt(message); // reads odom_x
    std::advance(skipIt, 2); // odom_y odom_theta

    double timestamp = 0.0;
    message >> timestamp;

    int64_t frameTimestamp = utils::sec_to_usec(timestamp);

    odom.timestamp = frameTimestamp;
    scan.timestamp = frameTimestamp;

    int64_t timeDelta = frameTimestamp - previousLaserTimestamp;

    // If the laser timestamp isn't changing, then create new ones ourselves
    if(timeDelta == 0)
    {
        previousAssignedTimestamp += 200000;   // assume 5Hz for non-changing data, as Carmen logs seem around that
        frameTimestamp = previousAssignedTimestamp;
    }
    else if(timeDelta > 0)
    {
        previousAssignedTimestamp = frameTimestamp;
        previousLaserTimestamp = frameTimestamp;
    }
    else // if(timeDelta < 0)
    {
        std::cout << "WARNING: Laser time went backwards! Amount:" << (-timeDelta / 1000) << '\n';
        return;
    }

    if(havePreviousPose)
    {
        odom.translation = std::sqrt(std::pow((odom.x-previousPose.x), 2) + std::pow((odom.y-previousPose.y), 2));
        odom.rotation    = angle_diff(odom.theta, previousPose.theta);
    }
    else
    {
        odom.translation = 0.0;
        odom.rotation = 0.0;
    }

    previousPose.x     = odom.x;
    previousPose.y     = odom.y;
    previousPose.theta = odom.theta;
    havePreviousPose = true;

    frame.haveLaser    = true;
    frame.haveOdometry = true;
    frame.haveIMU      = false;

    frame.timestamp = frameTimestamp;

#ifdef DEBUG_MESSAGES
    std::cout<<"Laser time:"<<frame.timestamp<<" timestamp:"<<timestamp<<'\n';
#endif

    addFrame(frame);
}


void CarmenReader::processLaserMessage(std::istringstream& message)
{
    /*
     * The format for the new laser message is:
     *
     * ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode
     *  num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta
     *  robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis
     *  timestamp hostname log_time
     */
    data_frame_t frame;

    polar_laser_scan_t& scan = frame.scan;

    int laserType = 0;
    message >> laserType;

    message >> scan.startAngle;

    auto skipIt = std::istream_iterator<double>(message);   // creating iterator does one deference

    message >> scan.angularResolution;
    message >> scan.maxRange;

    skipIt = std::istream_iterator<double>(message); // accuracy
    std::advance(skipIt, 1); // remission_mode

    message >> scan.numRanges;
    scan.ranges.resize(scan.numRanges);
    std::copy_n(std::istream_iterator<double>(message), scan.numRanges, scan.ranges.begin());

    scan.laserId = 0;
    scan.scanPeriod = 1.0 / 75.0;       // assume laser is a SICK @ 75Hz

    // Skip reading remissions
    int numRemissions = 0;
    message >> numRemissions;
    if (numRemissions > 0)
    {
        skipIt = std::istream_iterator<double>(message);
        std::advance(skipIt, numRemissions - 1);
    }

    // store laser pose in the offset
    message >> scan.offset.x;
    message >> scan.offset.y;
    message >> scan.offset.theta;

    odometry_t& odom = frame.odometry;

    message >> odom.x;
    message >> odom.y;
    message >> odom.theta;

    // then subtract off the robot pose to get the actual laser offset
    scan.offset.x = odom.x - scan.offset.x;
    scan.offset.y = odom.y - scan.offset.y;
    scan.offset.theta = angle_diff(odom.theta, scan.offset.theta);

    // Skip reading safety, velocity
    skipIt = std::istream_iterator<double>(message); // tv
    std::advance(skipIt, 4); // rv, safety info

    double timestamp = 0.0;
    message >> timestamp;

    int64_t frameTimestamp = utils::sec_to_usec(timestamp);

    odom.timestamp = frameTimestamp;
    scan.timestamp = frameTimestamp;

    int64_t timeDelta = frameTimestamp - previousLaserTimestamp;

    // If the laser timestamp isn't changing, then create new ones ourselves
    if(timeDelta == 0)
    {
        previousAssignedTimestamp += 200000;   // assume 5Hz for non-changing data, as Carmen logs seem around that
        previousLaserTimestamp = previousAssignedTimestamp;
        frameTimestamp = previousAssignedTimestamp;
    }
    else if(timeDelta > 0)
    {
        previousAssignedTimestamp = frameTimestamp;
        previousLaserTimestamp = frameTimestamp;
    }
    else // if(timeDelta < 0)
    {
        std::cout << "WARNING: Laser time went backwards! Amount:" << (-timeDelta / 1000) << '\n';
        return;
    }

    if(havePreviousPose)
    {
        odom.translation = std::sqrt(std::pow((odom.x-previousPose.x), 2) + std::pow((odom.y-previousPose.y), 2));
        odom.rotation    = angle_diff(odom.theta, previousPose.theta);
    }
    else
    {
        odom.translation = 0.0;
        odom.rotation = 0.0;
    }

    previousPose.x     = odom.x;
    previousPose.y     = odom.y;
    previousPose.theta = odom.theta;
    havePreviousPose = true;

    frame.haveLaser    = true;
    frame.haveOdometry = true;
    frame.haveIMU      = false;

    frame.timestamp = frameTimestamp;

#ifdef DEBUG_MESSAGES
    std::cout<<"Laser time:"<<frame.timestamp<<" timestamp:"<<timestamp<<'\n';
#endif

    addFrame(frame);
}


void CarmenReader::readParamMessage(std::istringstream& message)
{
    // Ignore all params for now and hard-code for the Killian Court dataset
    std::cout << "Param message:" << message.str() << '\n';
    maxLaserRange = 40.0f;
}

} // namespace logplayer
} // namespace vulcan
