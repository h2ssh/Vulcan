/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     event_log_runner.cpp
* \author   Collin Johnson
*
* Definition of EventLogRunner.
*/

#include "hssh/global_topological/utils/event_log_runner.h"
#include "hssh/global_topological/director.h"
#include "hssh/global_topological/commands/save_topo_slam_data.h"
#include "hssh/local_topological/evaluation/stability_log.h"
#include "system/module_communicator.h"
#include "utils/serialized_file_io.h"
#include <iostream>

namespace vulcan
{
namespace hssh
{

EventLogRunner::EventLogRunner(const std::string& filename)
{
    AreaStabilityLog log(filename);
    events_.insert(events_.end(), log.beginEvent(), log.endEvent());
    poses_.insert(poses_.end(), log.beginLocal(), log.endLocal());
}


EventLogRunner::EventLogRunner(const std::string& eventName, const std::string& poseName)
{
    if(!utils::load_serializable_from_file(eventName, events_))
    {
        std::cerr << "ERROR: EventLogRunner: Failed to load events from " << eventName << '\n';
    }

    if(!utils::load_serializable_from_file(poseName, poses_))
    {
        std::cerr << "ERROR: EventLogRunner: Failed to load poses from " << poseName << '\n';
    }
}


void EventLogRunner::run(GlobalTopoDirector& director, bool interactive)
{
    using EventIter = LocalAreaEventVec::const_iterator;
    using PoseIter = std::vector<LocalPose>::const_iterator;

    EventIter evIt = events_.begin();
    PoseIter poseIt = poses_.begin();

    system::ModuleCommunicator communicator;
    LocalAreaEventVec event(1);     // events are passed in as vectors of events, but we're just doing one at a time

    // On each iteration, save the map data so it can be easily loaded for debugging
    auto saveTree = std::make_shared<SaveTopoSlamDataCommand>(TopoSlamDataType::tree_of_maps, "current_tree_of_maps.tom", "event log runner");
    auto saveCache = std::make_shared<SaveTopoSlamDataCommand>(TopoSlamDataType::map_cache, "current_map_cache.mmc", "event log runner");

    while((evIt != events_.end()) || (poseIt != poses_.end()))
    {
        bool sendEvent = (poseIt == poses_.end())   // if there are no poses, then only events still exist
            || ((evIt != events_.end()) // otherwise, if an event and it is before next timestamp, it should be next
                && ((*evIt)->timestamp() < poseIt->timestamp()));

        if(sendEvent)
        {
            // If interactive mode, wait until the user hits enter, then keep going
            if(interactive)
            {
                std::cin.ignore();
            }

            event[0] = *evIt;
            director.handleData(event, "LOCAL_EVENT");
            ++evIt;
        }
        else
        {
            director.handleData(*poseIt, "LOCAL_POSE");
            ++poseIt;
        }

        director.runUpdate(communicator);

        // For each event, save the corresponding state for debugging
        if(sendEvent)
        {
            director.handleData(saveTree, "HSSH_GLOBAL_TOPO_COMMAND");
            director.handleData(saveCache, "HSSH_GLOBAL_TOPO_COMMAND");
        }
    }

    // Once the log ends, shutdown the director
    director.shutdown(communicator);
}

} // namespace hssh
} // namespace vulcan
