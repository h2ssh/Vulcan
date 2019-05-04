/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     player_reader.cpp
* \author   Collin Johnson
* 
* Definition of PlayerReader for reading Player/Stage log files.
*/

#include <logging/logplayer/player_reader.h>

namespace vulcan 
{
namespace logplayer
{
    
PlayerReader::PlayerReader(const data_channels_t& channels)
: LogReader(channels)
{
    
}


bool PlayerReader::convertLogToFrames(const std::string & filename)
{
    return false;
}

} // namespace logplayer
} // namespace vulcan
