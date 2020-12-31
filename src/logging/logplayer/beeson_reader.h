/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     beeson_reader.h
 * \author   Collin Johnson
 *
 * Declaration of BeesonReader.
 */

#ifndef LOGPLAYER_BEESON_READER_H
#define LOGPLAYER_BEESON_READER_H

#include "logging/logplayer/log_reader.h"
#include "logging/logplayer/params.h"

namespace vulcan
{
namespace logplayer
{

/**
 * BeesonReader is a LogReader implementation capable of reading logs recorded by Patrick Beeson
 * and posted in the Radish repository fo robot logs.
 */
class BeesonReader : public LogReader
{
public:
    /**
     * Constructor for BeesonReader.
     *
     * \param    channels            Channels on which to transmit the data
     */
    BeesonReader(const data_channels_t& channels);

private:
    // LogReader interface
    bool convertLogToFrames(const std::string& filename) override;
};

}   // namespace logplayer
}   // namespace vulcan

#endif   // LOGPLAYER_BEESON_READER_H
