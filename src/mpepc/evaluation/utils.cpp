/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* @file
* @author   Collin Johnson
*
* Definition of utility types and functions for evaluating the performance of MPEPC.
*/

#include "mpepc/evaluation/utils.h"
#include "utils/serialized_file_io.h"
#include <fstream>
#include <sstream>

namespace vulcan
{
namespace mpepc
{

const std::string kRegularMPEPC("regular");
const std::string kSocialMPEPC("social");
const std::string kUnknownMPEPC("unknown");


std::vector<ResultsLog> load_results_logs(const std::string& filename)
{
    std::vector<ResultsLog> logs;

    std::ifstream in(filename);

    if(!in.is_open())
    {
        std::cerr << "ERROR: Failed to read logs file: " << filename << '\n';
        return logs;
    }

    for(std::string line; std::getline(in, line);)
    {
        std::istringstream logIn(line);
        std::string version;
        std::string logName;
        std::string mapName;
        logIn >> version >> logName >> mapName;

        // If there's no enough info, then done early.
        if(logName.empty() || mapName.empty())
        {
            break;
        }

        ResultsLog log;
        log.version = name_to_version(version);
        log.logName = logName;
        log.ltmName = mapName;
        if (utils::load_serializable_from_file(mapName, log.map)) {
            logs.push_back(log);

            std::cout << "Loaded log " << logName << ' ' << mapName << '\n';
        }
    }

    return logs;
}


MPEPCVersion name_to_version(const std::string& versionStr)
{
    if(versionStr == kRegularMPEPC)
    {
        return MPEPCVersion::regular;
    }
    else if(versionStr == kSocialMPEPC)
    {
        return MPEPCVersion::social;
    }

    return MPEPCVersion::unknown;
}


std::string version_to_name(MPEPCVersion version)
{
    switch(version)
    {
    case MPEPCVersion::regular:
        return kRegularMPEPC;
    case MPEPCVersion::social:
        return kSocialMPEPC;
    default:
        return kUnknownMPEPC;
    }

    return kUnknownMPEPC;
}


std::string version_to_public_name(MPEPCVersion version)
{
    switch(version)
    {
    case MPEPCVersion::regular:
        return "MPEPC";
    case MPEPCVersion::social:
        return "SA-MPEPC";
    default:
        assert(0);
    }

    assert(false);
    return "ERROR";
}

} // namespace mpepc
} // namespace vulcan
