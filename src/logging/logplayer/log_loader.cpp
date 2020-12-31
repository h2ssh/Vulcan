/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     log_loader.cpp
 * \author   Collin Johnson
 *
 * Definition of LogLoader.
 */

#include "logging/logplayer/log_loader.h"

namespace vulcan
{
namespace logplayer
{

inline bool is_carmen_log(const std::string& extension)
{
    return extension == "clf";
}


inline bool is_saphira_log(const std::string& extension)
{
    return extension == "slf";
}


LogLoader::LogLoader(const data_channels_t& channels)
{
    availableReaders = load_log_readers(channels);
}


std::shared_ptr<LogReader> LogLoader::loadLog(const std::string& filename, log_type_t type)
{
    auto logReader = availableReaders.find(type);

    if (logReader != availableReaders.end()) {
        logReader->second->openLog(filename);
        return logReader->second;
    }

    size_t extensionStart = filename.rfind('.');
    assert(extensionStart != std::string::npos);

    std::string extension(filename.substr(extensionStart + 1));

    if (is_carmen_log(extension)) {
        std::cout << "INFO:LogLoader: Auto-detected type for " << filename << " is LOG_CARMEN.\n";
        logReader = availableReaders.find(LOG_CARMEN);
        logReader->second->openLog(filename);
        return logReader->second;
    } else if (is_saphira_log(extension)) {
        std::cout << "INFO:LogLoader: Auto-detected type for " << filename << " is LOG_SAPHIRA.\n";
        logReader = availableReaders.find(LOG_SAPHIRA);
        logReader->second->openLog(filename);
        return logReader->second;
    } else {
        std::cerr << "ERROR:LogLoader: Unknown file extension " << extension << '\n';
    }

    return std::shared_ptr<LogReader>();
}

}   // namespace logplayer
}   // namespace vulcan
