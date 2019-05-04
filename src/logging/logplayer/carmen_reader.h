/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     carmen_reader.h
* \author   Collin Johnson
*
* Declaration of CarmenReader.
*/

#ifndef LOGPLAYER_CARMEN_READER_H
#define LOGPLAYER_CARMEN_READER_H

#include <logging/logplayer/log_reader.h>
#include <logging/logplayer/params.h>
#include <iosfwd>

namespace vulcan
{
namespace logplayer
{

/**
* CarmenReader is an implementation of the LogReader interface that is capable of reading
* log files recorded by the CARMEN robot toolkit. The reader uses the CARMEN format, so logfiles
* must be in that format to be parsed correctly.
*
* The types supported by the CARMEN log reader are:
*
*   PARAM : robot_length, robot_width, robot_frontlaser_offset, robot_front_laser_max
*   ODOM
*   FLASER
*   ROBOTLASER1
*
* NOTE: Odometry is also read from FLASER, thus odometry_t will always be associated with
*       a polar_laser_scan_t for carmen logs.
*/
class CarmenReader : public LogReader
{
public:

    /**
    * Constructor for CarmenReader.
    *
    * \param    channels            Channels on which to send the data
    */
    CarmenReader(const data_channels_t& channels);

private:

    enum carmen_message_type_t
    {
        PARAM_MESSAGE,
        OLD_LASER_MESSAGE,
        ROBOT_LASER_MESSAGE,
        UNKNOWN_MESSAGE,
    };

    // LogReader interface
    bool convertLogToFrames(const std::string& filename) override;

    carmen_message_type_t messageTypeFromString (const std::string& typeString);
    void                  processMessage        (carmen_message_type_t type, std::istringstream& message);
    void                  processOldLaserMessage(std::istringstream& message);
    void                  processLaserMessage   (std::istringstream& message);
    void                  readParamMessage      (std::istringstream& message);

    pose_t  previousPose;
    bool    havePreviousPose;
    int64_t previousLaserTimestamp;
    int64_t previousAssignedTimestamp;

    float maxLaserRange;
};

} // namespace logplayer
} // namespace vulcan

#endif // LOGPLAYER_CARMEN_READER_H
