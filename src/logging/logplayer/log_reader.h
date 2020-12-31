/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_reader.h
* \author   Collin Johnson
*
* Declaration of LogReader interface and factory for creating the various readers.
*/

#ifndef LOGPLAYER_LOG_READER_H
#define LOGPLAYER_LOG_READER_H

#include "logging/logplayer/params.h"
#include "core/imu_data.h"
#include "core/odometry.h"
#include "core/laser_scan.h"
#include <map>
#include <memory>
#include <string>
#include <vector>


namespace vulcan
{
namespace system { class ModuleCommunicator; }

namespace logplayer
{

class  LogReader;

enum log_type_t
{
    LOG_CARMEN,
    LOG_BEESON,
    LOG_PLAYER,
    LOG_SAPHIRA,
    AUTO_DETECT_LOG_TYPE,
};

/**
* load_log_readers loads instances of LogReader implementations for each log_type_t.
*
* \param    channels            Channels on which the various types of data are to be transmitted
* \return   A map from log_type_t->LogReader instance, so the appropriate reader can be used for a given log type.
*/
std::map<log_type_t, std::shared_ptr<LogReader>> load_log_readers(const data_channels_t& channels);

/**
* LogReader is an abstract base class for classes capable of reading log files. A log is some file containing
* frames of data. A data frame is considered to be all data occurring at a single point in time. Thus,
* a LogReader will be asked to give the timestamps for a log and to transmit the data occurring within
* a single log frame. A LogReader implementation is responsible for opening and parsing a log file into a
* series of data_frame_t which are then used by the LogReader base class for data transmission.
*
* sendFrame() will be called by the LogPlayer at the appropriate time, based on the current playback
* speed. The LogReader only needs to care about transmitting the data, but not when that data is to be
* transmitted.
*/
class LogReader
{
public:

    /**
    * Constructor for LogReader.
    *
    * \param    channels            Channels on which to transmit data
    */
    LogReader(const data_channels_t& channels);

    /**
    * openLog opens the specified log file. If the log is the wrong format or does not exist,
    * then false is returned and future calls to sendFrame() will produce nothing.
    *
    * \param    filename            Name of the log to be opened
    * \return   True if the log opened and is ready for playback. False otherwise.
    */
    bool openLog(const std::string& filename);

    /**
    * getTimestamps retrieves the timestamps for the frames in the log. A timestamp is measured
    * in microseconds.
    *
    * \return   Sequential timestamps for the data in the log.
    */
    std::vector<int64_t> getTimestamps(void) const;

    /**
    * sendFrame transmits the requested frame using the provided DataTransmitter instance.
    *
    * \param    index           Index of the frame to send
    * \param    communicator    Communicator to use for sending out the data
    * \return   True if the frame was sent. False otherwise.
    */
    bool sendFrame(uint32_t index, system::ModuleCommunicator& communicator) const;

protected:

    struct data_frame_t
    {
        int64_t timestamp = 0;
        bool    haveIMU = false;
        bool    haveLaser = false;
        bool    haveOdometry = false;

        odometry_t       odometry;
        imu_data_t       imu;
        polar_laser_scan_t scan;

        bool operator<(const data_frame_t& rhs) const { return timestamp < rhs.timestamp; }
    };
    
    /**
    * convertLogToFrames is the method used by the LogReader instance to turn the loaded log file into data frames that
    * can be sent.
    * 
    * \param    filename        File to be converted into frames
    * \return   True if the file was understood.
    */
    virtual bool convertLogToFrames(const std::string& filename) = 0;

    void addFrame(const data_frame_t& frame);
    void sortFramesByTimestamp(void);

private:

    std::vector<data_frame_t> frames;
    data_channels_t           channels;
};

}
}

#endif // LOGPLAYER_LOG_READER_H
