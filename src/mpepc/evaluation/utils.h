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
 * Declaration of utility types and functions for evaluating the performance of MPEPC.
 */

#ifndef MPEPC_EVALUATION_UTILS_H
#define MPEPC_EVALUATION_UTILS_H

#include "hssh/local_topological/local_topo_map.h"
#include <string>
#include <vector>

namespace vulcan
{
namespace mpepc
{

enum class MPEPCVersion
{
    regular,
    social,
    unknown,
};

struct ResultsLog
{
    MPEPCVersion version;
    std::string logName;
    std::string ltmName;
    hssh::LocalTopoMap map;
};


std::vector<ResultsLog> load_results_logs(const std::string& filename);
MPEPCVersion name_to_version(const std::string& versionStr);
std::string version_to_name(MPEPCVersion version);
std::string version_to_public_name(MPEPCVersion version);


}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_EVALUATION_UTILS_H
