/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_reader.cpp
* \author   Collin Johnson
*
* Definition of LogReader and factory for loading the various log readers.
*/

#include <logging/logplayer/log_reader.h>
#include <logging/logplayer/beeson_reader.h>
#include <logging/logplayer/carmen_reader.h>
#include <logging/logplayer/player_reader.h>
#include <logging/logplayer/saphira_reader.h>
#include <system/module_communicator.h>
#include <algorithm>


namespace vulcan
{
namespace logplayer
{

std::map<log_type_t, std::shared_ptr<LogReader>> load_log_readers(const data_channels_t& channels)
{
    std::map<log_type_t, std::shared_ptr<LogReader>> readers;

    readers.insert(std::make_pair(LOG_CARMEN, std::shared_ptr<LogReader>(new CarmenReader(channels))));
    readers.insert(std::make_pair(LOG_BEESON, std::shared_ptr<LogReader>(new BeesonReader(channels))));
    readers.insert(std::make_pair(LOG_PLAYER, std::shared_ptr<LogReader>(new PlayerReader(channels))));
    readers.insert(std::make_pair(LOG_SAPHIRA, std::shared_ptr<LogReader>(new SaphiraReader(channels))));

    return readers;
}


LogReader::LogReader(const data_channels_t& channels)
: channels(channels)
{
}


bool LogReader::openLog(const std::string& filename)
{
    // When reading a new log, clear out all old information of any previously loaded log
    frames.clear();

    // The subclass reads the actual log information
    return convertLogToFrames(filename);
}


std::vector<int64_t> LogReader::getTimestamps(void) const
{
    std::vector<int64_t> timestamps;

    std::for_each(frames.begin(), frames.end(), [&](const data_frame_t& frame) { timestamps.push_back(frame.timestamp); });

    return timestamps;
}


bool LogReader::sendFrame(uint32_t index, system::ModuleCommunicator& communicator) const
{
    if(index >= frames.size())
    {
        return false;
    }

    const data_frame_t& frame = frames[index];

    if(frame.haveIMU)
    {
        communicator.sendMessage(frame.imu);
    }

    if(frame.haveOdometry)
    {
        communicator.sendMessage(frame.odometry);
    }

    if(frame.haveLaser)
    {
        communicator.sendMessage(frame.scan, channels.laserChannel);
    }

    return true;
}


void LogReader::addFrame(const data_frame_t& frame)
{
    frames.push_back(frame);
}


void LogReader::sortFramesByTimestamp(void)
{
    std::sort(frames.begin(), frames.end());
}

}
}
