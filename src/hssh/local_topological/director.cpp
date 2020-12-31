/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topology_director.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalTopologyDirector.
 */

#include "hssh/local_topological/director.h"
#include "core/pose_distribution.h"
#include "hssh/local_metric/commands/truncate_lpm.h"
#include "hssh/local_topological/command.h"
#include "hssh/local_topological/debug_info.h"
#include "hssh/local_topological/location.h"
#include "hssh/local_topological/params.h"
#include "hssh/local_topological/small_scale_space_boundary.h"
#include "system/debug_communicator.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include "utils/serialized_file_io.h"
#include "utils/timestamp.h"
#include <iostream>

#define DEBUG_TIMING

namespace vulcan
{
namespace hssh
{

LocalTopologyDirector::LocalTopologyDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: areaDetector_(config, commandLine, commandLine.argumentValue(kDirectoryArg) + commandLine.argumentValue(kMapNameArg))
, eventDetector_(config)
, boundary_(create_small_scale_space_boundary(local_topology_params_t(config).smallScaleSpaceType))
, mode_(LocalTopoMode::event_detection)
, failedLpmIndex_(0)
, haveNewLpm_(false)
, haveInitializedPose_(false)
, shouldSaveEvents_(commandLine.argumentExists(kSaveEventsArg))
, eventsFilename_(commandLine.argumentValue(kSaveEventsArg))
, shouldSaveMap_(commandLine.argumentExists(kSaveMapArg))
, mapFilename_(commandLine.argumentValue(kSaveMapArg))
, inputTrigger_(false)
{
    const std::string kDefaultEventsFile("local_topo_events.log");
    const std::string kDefaultMapFile("local_topo_map.ltm");

    if (shouldSaveEvents_ && eventsFilename_.empty()) {
        std::cerr
          << "WARNING! LocalTopologyDirector: Specified to save the events, but no filename was specified. Using "
          << " the default name: " << kDefaultEventsFile << ". Make sure to copy this file after exiting so the "
          << "events aren't lost when you run local_topo_hssh again.\n";
        eventsFilename_ = kDefaultEventsFile;
    } else if (shouldSaveEvents_) {
        std::cout << "INFO: LocalTopologyDirector: Saving LocalAreaEvents to " << eventsFilename_ << " on exit.\n";
    }

    if (shouldSaveMap_ && mapFilename_.empty()) {
        std::cerr << "WARNING! LocalTopologyDirector: Specified to save the map, but no filename was specified. Using "
                  << " the default name: " << kDefaultMapFile << ". Make sure to copy this file after exiting so the "
                  << "map isn't lost when you run local_topo_hssh again.\n";
        mapFilename_ = kDefaultMapFile;
    } else if (shouldSaveMap_) {
        std::cout << "INFO: LocalTopologyDirector: Saving LocalTopoMap to " << mapFilename_ << " on exit.\n";
    }
}


LocalTopoMap LocalTopologyDirector::oneShotLabeling(const LocalPerceptualMap& lpm)
{
    AreaDetectorResult result = areaDetector_.detectAreas(LocalPose(), lpm);

    if (result.result == LabelingError::success) {
        return *result.map;
    } else {
        return LocalTopoMap();
    }
}


LocalTopologyDirector::~LocalTopologyDirector(void)
{
}


void LocalTopologyDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<LocalPerceptualMap>(this);
    communicator.subscribeTo<LocalPose>(this);
    communicator.subscribeTo<std::shared_ptr<LocalTopoCommand>>(this);
}


system::TriggerStatus LocalTopologyDirector::waitForTrigger(void)
{
    if (inputTrigger_.timedWait(100)) {
        return system::TriggerStatus::not_ready;
    }

    inputTrigger_.setPredicate(false);

    // Once awoken, make sure data should be processed. This only won't be the case when the reference frame
    // changes and the LPM arrives before the next pose and processing wants to get started
    // Don't need to worry about an infinite loop because the reference frame changes much less frequently than
    // maps are sent out
    do {
        if (lpm.hasData()) {
            lpm.swapBuffers();
            haveNewLpm_ = true;
        }

        if (currentPose.hasData()) {
            currentPose.swapBuffers();
        }

        usleep(500);
    } while (!shouldProcessData());

    return system::TriggerStatus::ready;
}


system::UpdateStatus LocalTopologyDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    searchPose_ = currentPose.read();

    findAreas(communicator);
    if (mode_ == LocalTopoMode::event_detection) {
        findEvents(communicator);
        maintainSmallScaleSpace(communicator);
    } else {
        events_.clear();
    }

    // local_topo_hssh always runs
    return system::UpdateStatus::running;
}


void LocalTopologyDirector::shutdown(system::ModuleCommunicator& communicator)
{
    if (shouldSaveEvents_) {
        if (!utils::save_serializable_to_file(eventsFilename_, allEvents_)) {
            std::cerr << "ERROR! LocalTopologyDirector: Failed to save the events to " << eventsFilename_ << '\n';
        } else {
            std::cout << "INFO: LocalTopologyDirector: Successfully saved the events to " << eventsFilename_ << '\n';
        }
    }

    if (shouldSaveMap_) {
        if (!utils::save_serializable_to_file(mapFilename_, map_)) {
            std::cerr << "ERROR! LocalTopologyDirector: Failed to save the map to " << mapFilename_ << '\n';
        } else {
            std::cout << "INFO: LocalTopologyDirector: Successfully saved the map to " << mapFilename_ << '\n';
        }
    }
}


void LocalTopologyDirector::handleData(const LocalPerceptualMap& grid, const std::string& channel)
{
    lpm.write(grid);
    referenceFrameIndex = grid.getReferenceFrameIndex();

    if (shouldProcessData()) {
        inputTrigger_.setPredicate(true);
        inputTrigger_.broadcast();
    }
}


void LocalTopologyDirector::handleData(const LocalPose& pose, const std::string& channel)
{
    currentPose.write(pose);

    {
        utils::AutoMutex autoLock(eventDetectionLock_);
        eventDetector_.addPose(pose);
    }

    if (shouldProcessData() || !haveInitializedPose_) {
        inputTrigger_.setPredicate(true);
        inputTrigger_.broadcast();

        haveInitializedPose_ = true;
    }
}


void LocalTopologyDirector::handleData(const std::shared_ptr<LocalTopoCommand>& command, const std::string& channel)
{
    // Just change the mode
    mode_ = command->mode();
}


bool LocalTopologyDirector::shouldProcessData(void)
{
    // Can process as long as the lpm and pose have the same reference frame, otherwise not safe to start yet
    return currentPose.read().referenceFrameIndex() == lpm.read().getReferenceFrameIndex();
}


void LocalTopologyDirector::findAreas(system::ModuleCommunicator& communicator)
{
    // Only update the areas if a new LPM has arrived, otherwise the previous data is up-to-date
    if (!haveNewLpm_) {
        return;
    }

    AreaDetectorResult result = areaDetector_.detectAreas(searchPose_, lpm.read());

    if (result.result == LabelingError::success) {
        map_ = *result.map;

        // Only send a map at most once a second or it probably won't ever be received elsewhere
        int64_t now = utils::system_time_us();
        if (now - lastMapSentTimeUs_ > 1000000) {
            communicator.sendMessage(map_);
            lastMapSentTimeUs_ = now;
        }

        utils::save_serializable_to_file("local_topo.lpm", lpm.read());
        utils::save_serializable_to_file("current.ltm", map_);
    } else {
        std::ostringstream filename;
        filename << "failed_lpm_" << failedLpmIndex_ << ".lpm";
        ++failedLpmIndex_;
        std::cerr << "WARNING: Failed to classify LPM. Saving to file as " << filename.str() << '\n';
        utils::save_serializable_to_file(filename.str(), lpm.read());

        // If the error is due to an exit area or transition being incorrect, then the event detector needs to
        // informed so appropriate action can be taken
    }

    system::DebugCommunicator debug(communicator);
    areaDetector_.sendDebug(debug);

    haveNewLpm_ = false;
}


void LocalTopologyDirector::findEvents(system::ModuleCommunicator& communicator)
{
    events_.clear();

    {
        utils::AutoMutex autoLock(eventDetectionLock_);
        events_ = eventDetector_.detectEvents(map_);
    }

    if (!events_.empty()) {
        communicator.sendMessage(events_);
    }

    if (shouldSaveEvents_ && !events_.empty()) {
        allEvents_.insert(allEvents_.end(), events_.begin(), events_.end());
    }

    areaDetector_.processAreaEvents(events_);
    location_ = localizer_.localizeRobot(events_, map_);
    communicator.sendMessage(location_);
}


void LocalTopologyDirector::maintainSmallScaleSpace(system::ModuleCommunicator& communicator)
{
    // The boundary doesn't need to be updated if no new events have occurred
    if (events_.empty()) {
        return;
    }

    auto newBoundary = boundary_->computeBoundary(events_, map_, searchPose_, lpm);

    // If there's a new boundary, then send out the appropriate truncation event
    if (newBoundary) {
        std::shared_ptr<LocalMetricCommand> truncateCmd =
          std::make_shared<TruncateLpmCommand>("local_topo_hssh", *newBoundary);
        communicator.sendMessage(truncateCmd);

        std::cout << "INFO: LocalTopologyDirector: Updating the boundary to " << *newBoundary << '\n';
    }
}

}   // namespace hssh
}   // namespace vulcan
