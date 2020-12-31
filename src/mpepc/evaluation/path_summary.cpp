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
* Definition of PathSummary.
*/

#include "mpepc/evaluation/path_summary.h"
#include "mpepc/evaluation/mpepc_log.h"
#include "mpepc/evaluation/utils.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace mpepc
{

utils::PoseTrace extract_trace_from_log(MPEPCLog& log);


PathSummary::PathSummary(const std::string& filename)
{
    auto resultsLogs = load_results_logs(filename);

    for(auto& results : resultsLogs)
    {
        MPEPCLog log(results.logName);
        auto trace = extract_trace_from_log(log);

        switch(results.version)
        {
        case MPEPCVersion::regular:
            regularPoses_.emplace_back(std::move(trace));
            break;
        case MPEPCVersion::social:
            socialPoses_.emplace_back(std::move(trace));
            break;
        case MPEPCVersion::unknown:
        default:
            std::cerr << "ERROR: Unknown log version type for " << results.logName << '\n';
            break;
        }
    }

    std::cout << "INFO: PathSummary: Loaded traces: \n"
        << version_to_name(MPEPCVersion::regular) << " : " << regularPoses_.size()
        << version_to_name(MPEPCVersion::social) << " : " << socialPoses_.size() << '\n';
}


utils::PoseTrace extract_trace_from_log(MPEPCLog& log)
{
    utils::PoseTrace trace(0.025f, 10000000);
    log.loadAll();

    for(auto& motion : boost::make_iterator_range(log.beginMotionState(), log.endMotionState()))
    {
        trace.addPose(motion.pose);
    }

    return trace;
}

} // namespace mpepc
} // namespace vulcan
