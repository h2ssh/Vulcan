/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "logging/logplayer/beeson_reader.h"

namespace vulcan
{
namespace logplayer
{

BeesonReader::BeesonReader(const data_channels_t& channels)
                    : LogReader(channels)
{
}


bool BeesonReader::convertLogToFrames(const std::string& filename)
{
    return false;
}

}
}
