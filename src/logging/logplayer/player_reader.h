/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     player_reader.h
 * \author   Collin Johnson
 *
 * Declaration of PlayerReader for reading Player/Stage log files -- .plf
 */

#ifndef LOGGING_LOGPLAYER_PLAYER_READER_H
#define LOGGING_LOGPLAYER_PLAYER_READER_H

#include "logging/logplayer/log_reader.h"

namespace vulcan
{
namespace logplayer
{

/**
 * PlayerReader
 */
class PlayerReader : public LogReader
{
public:
    /**
     * Constructor for PlayerReader.
     *
     * \param    channels        Channels on which to publish sensor data
     */
    PlayerReader(const data_channels_t& channels);

private:
    // LogReader interface
    bool convertLogToFrames(const std::string& filename) override;
};

}   // namespace logplayer
}   // namespace vulcan

#endif   // LOGGING_LOGPLAYER_PLAYER_READER_H
