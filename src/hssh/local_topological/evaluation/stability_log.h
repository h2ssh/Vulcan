/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     stability_log.h
 * \author   Collin Johnson
 *
 * Declaration of AreaStabilityLog.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_LOG_H
#define HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_LOG_H

#include "hssh/global_metric/pose.h"
#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/event.h"

namespace vulcan
{
namespace hssh
{

/**
 * AreaStabilityLog is a utility for loading the contents of a sensor log into RAM. AreaStabilityLog exists to avoid
 * needing to use a logplayer to read all the sensor data from a log for processing, be it for doing calculations or
 * converting to a Matlab-friendly format.
 *
 * The data in AreaStabilityLog is sorted in order of ascending timestamps, so begin->timestamp < end->timestamp.
 *
 * A AreaStabilityLog instance is created by providing an LCM log. This log will be read and converted into the
 * appropriate data types. The various types can then be iterated over as desired.
 *
 * IMPORTANT: Calling any of the addXXXX methods will invalidate iterators of the same type. In general, you should just
 * provide the log file to the constructor and never need to touch the add methods.
 */
class AreaStabilityLog
{
public:
    using local_iterator = std::vector<LocalPose>::const_iterator;
    using global_iterator = std::vector<GlobalPose>::const_iterator;
    using event_iterator = std::vector<LocalAreaEventPtr>::const_iterator;

    /**
     * Default constructor for AreaStabilityLog.
     *
     * Create an empty log.
     */
    AreaStabilityLog(void) = default;

    /**
     * Constructor for AreaStabilityLog.
     *
     * Create a AreaStabilityLog containing all data in the provided LCM log.
     *
     * \param    logFilename         Name of the log to be loaded
     */
    AreaStabilityLog(const std::string& logFilename);

    // Disable copying, because these logs can be huge. Only moves are allowed for sanity's sake
    AreaStabilityLog(const AreaStabilityLog&) = delete;
    AreaStabilityLog& operator=(const AreaStabilityLog&) = delete;

    AreaStabilityLog(AreaStabilityLog&&) = default;
    AreaStabilityLog& operator=(AreaStabilityLog&&) = default;

    /**
     * name retrieves the name of the sensor log. It is just the end of the filename, not the full path.
     */
    std::string name(void) const { return name_; }


    // Iterator access for the data in the log
    std::size_t sizeLocal(void) const { return localPoses_.size(); }
    local_iterator beginLocal(void) const { return localPoses_.begin(); }
    local_iterator endLocal(void) const { return localPoses_.end(); }

    std::size_t sizeGlobal(void) const { return globalPoses_.size(); }
    global_iterator beginGlobal(void) const { return globalPoses_.begin(); }
    global_iterator endGlobal(void) const { return globalPoses_.end(); }

    std::size_t sizeEvent(void) const { return events_.size(); }
    event_iterator beginEvent(void) const { return events_.begin(); }
    event_iterator endEvent(void) const { return events_.end(); }

    // Add the various type of data to the log
    void addLocalPose(const LocalPose& pose);
    void addGlobalPose(const GlobalPose& pose);
    void addEvent(const LocalAreaEventPtr& event);

private:
    std::string name_;

    std::vector<LocalPose> localPoses_;
    std::vector<GlobalPose> globalPoses_;
    std::vector<LocalAreaEventPtr> events_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_LOG_H
