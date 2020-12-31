/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     log_loader.h
 * \author   Collin Johnson
 *
 * Declaration of LogLoader.
 */

#ifndef LOGPLAYER_LOG_LOADER_H
#define LOGPLAYER_LOG_LOADER_H

#include "logging/logplayer/log_reader.h"
#include <boost/shared_ptr.hpp>
#include <map>

namespace vulcan
{
namespace logplayer
{

/**
 * LogLoader accepts a log filename and creates the appropriate LogReader for further
 * processing of the file. The log loader accepts a filename and an optional file type.
 * When no type is provided, the loader uses the file extension and then a brute force
 * search to find the appropriate reader for a log.
 */
class LogLoader
{
public:
    /**
     * Constructor for LogLoader.
     *
     * \param    channels            Channels the readers will use for sending frames
     */
    LogLoader(const data_channels_t& channels);

    /**
     * loadLog loads the provided log and returns the appropriate LogReader instance. If an appropriate
     * reader cannot be found, a null pointer is returned.
     *
     * In auto-detect mode, the behavior is to use the file extension to determine the appropriate
     * load reader to use.
     *
     * \param    filename            Name of the log file to be loaded
     * \param    type                Type of the log file (optional, default = auto detect)
     * \return   Instance of LogReader to use for the given file. Null if no reader exists.
     */
    std::shared_ptr<LogReader> loadLog(const std::string& filename, log_type_t type = AUTO_DETECT_LOG_TYPE);

private:
    std::map<log_type_t, std::shared_ptr<LogReader>> availableReaders;
};

}   // namespace logplayer
}   // namespace vulcan

#endif   // LOGPLAYER_LOG_LOADER_H
