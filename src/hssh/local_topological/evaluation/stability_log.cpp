/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     stability_log.cpp
 * \author   Collin Johnson
 *
 * Definition of AreaStabilityLog.
 */

#include "hssh/local_topological/evaluation/stability_log.h"
#include "system/module_communicator.h"
#include <algorithm>
#include <sstream>

namespace vulcan
{
namespace hssh
{

// A utility class to handle the interaction with the ModuleCommunicator.
class LCMAreaStabilityCallbacks
{
public:
    LCMAreaStabilityCallbacks(AreaStabilityLog& log) : log_(log) { }

    void handleData(const LocalPose& pose, const std::string& channel) { log_.addLocalPose(pose); }

    void handleData(const GlobalPose& pose, const std::string& channel) { log_.addGlobalPose(pose); }

    void handleData(const LocalAreaEventVec& events, const std::string& channel)
    {
        for (auto& e : events) {
            log_.addEvent(e);
        }
    }

private:
    AreaStabilityLog& log_;   // Log to add the data to
};


//////////////////////////////////////// AreaStabilityLog implementation ////////////////////////////////////////////
AreaStabilityLog::AreaStabilityLog(const std::string& logFilename) : name_(logFilename)
{
    // Load all values from the provided LCM log. The LCM callbacks instance that will interact with LCM and callback
    // to add the values to the sensor log.
    LCMAreaStabilityCallbacks callbacks(*this);

    // To create a log provider, we can use a special Url for LCM. The Url to use was found in the source for the
    // lcm-logplayer. The interesting command is setting speed=0, which causes the log to playback as fast as it can
    // be read, as opposed to waiting between messages. Info on speed=0 can be found in lcm/lcm_file.c.
    std::ostringstream logProviderUrl;
    logProviderUrl << "file://" << logFilename << "?speed=0";   // default mode is read mode, but have to read LCM code
                                                                // carefully to actually know that
    system::ModuleCommunicator comm(logProviderUrl.str(), "");

    // Subscribe to each message in the log
    comm.subscribeTo<LocalPose>(&callbacks);
    comm.subscribeTo<GlobalPose>(&callbacks);
    comm.subscribeTo<LocalAreaEventVec>(&callbacks);

    // Keep reading messages until the processIncoming doesn't return success, indicating that there's no data left
    while (comm.processIncoming() == 0)
        ;

    std::cout << "INFO: AreaStabilityLog: Loaded the following data from " << name_ << ":\n"
              << "Local Poses:  " << localPoses_.size() << '\n'
              << "Global Poses: " << globalPoses_.size() << '\n'
              << "Events:       " << events_.size() << '\n';
}


void AreaStabilityLog::addLocalPose(const LocalPose& pose)
{
    localPoses_.push_back(pose);
}


void AreaStabilityLog::addGlobalPose(const GlobalPose& pose)
{
    globalPoses_.push_back(pose);
}


void AreaStabilityLog::addEvent(const LocalAreaEventPtr& event)
{
    events_.push_back(event);
}

}   // namespace hssh
}   // namespace vulcan
