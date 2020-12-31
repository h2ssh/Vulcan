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
 * Declaration of PathSummary.
 */

#ifndef MPEPC_EVALUATION_PATH_SUMMARY_H
#define MPEPC_EVALUATION_PATH_SUMMARY_H

#include "utils/pose_trace.h"
#include <vector>

namespace vulcan
{
namespace mpepc
{

/**
 * A path summary loads a file with logs and maps and splits them into two sets of PoseTraces, one for SA-MPEPC logs and
 * one for MPEPC logs.
 *
 * Each collection contains one PoseTrace per log file. They can be iterated over separately for easier processing.
 */
class PathSummary
{
public:
    using const_iterator = std::vector<utils::PoseTrace>::const_iterator;

    /**
     * Create a new summary from a file describing the logs. The format is the same as used by social_norm_results:
     *
     *   type    log     ltm
     *
     * with type = { social, regular }
     *
     * @param    filename        Name of file with log information
     */
    PathSummary(const std::string& filename);

    // Iterators for each type of trace
    std::size_t sizeSocial(void) const { return socialPoses_.size(); }
    const_iterator beginSocial(void) const { return socialPoses_.begin(); }
    const_iterator endSocial(void) const { return socialPoses_.end(); }

    std::size_t sizeRegular(void) const { return regularPoses_.size(); }
    const_iterator beginRegular(void) const { return regularPoses_.begin(); }
    const_iterator endRegular(void) const { return regularPoses_.end(); }

private:
    std::vector<utils::PoseTrace> socialPoses_;
    std::vector<utils::PoseTrace> regularPoses_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_EVALUATION_PATH_SUMMARY_H
