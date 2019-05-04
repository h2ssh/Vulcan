/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.cpp
* \author   Collin Johnson
*
* Definition of GlobalTopoDirector.
*/

#include <hssh/global_topological/director.h>
#include <hssh/global_topological/topo_slam.h>
#include <hssh/global_topological/commands/serialization.h>
#include <system/debug_communicator.h>
#include <system/module_communicator.h>
#include <utils/auto_mutex.h>
#include <utils/stub.h>
#include <utils/timestamp.h>
#include <iostream>
#include <cassert>

#define DEBUG_TIMING

namespace vulcan
{
namespace hssh
{

GlobalTopoDirector::GlobalTopoDirector(std::unique_ptr<TopologicalSLAM> slam,
                                       const utils::CommandLine& commandLine,
                                       const utils::ConfigFile& config)
: params_(config)
, slam_(std::move(slam))
{
}


GlobalTopoDirector::~GlobalTopoDirector(void)
{
    // For std::unique_ptr
}


void GlobalTopoDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<LocalAreaEventVec>(&queue_);
    communicator.subscribeTo<LocalPose>(&queue_);
    communicator.subscribeTo<GlobalTopoCommandPtr>(&queue_);
    communicator.subscribeTo<LocalTopoMap>(&queue_);
}


system::TriggerStatus GlobalTopoDirector::waitForTrigger(void)
{
    return queue_.waitForData() ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus GlobalTopoDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    std::size_t numCommands = queue_.numCommands(); // set a maximum size on the commands to ensure that something
                                                    // sending a million commands doesn't hang the update

    // Process any available commands
    for(std::size_t n = 0; (n < numCommands) && queue_.hasNewCommand(); ++n)
    {
        auto command = queue_.popCommand();
        assert(command);
        processCommand(*command);
    }

    // Process updated pose information if available
    if(queue_.hasNewPose())
    {
        slam_->updatePose(queue_.popPose());
    }

    int64_t updateStart = utils::system_time_us();
    std::size_t numEvents = queue_.numEvents(); // set a maximum size on the commands to ensure that something
                                                // sending a million events doesn't hang the update
    auto localMap = queue_.popLocalMap();

    if(numEvents > 0)
    {
        std::cout << "Processing " << numEvents << " events.\n";
    }

    // Process any available events
    for(std::size_t n = 0; (n < numEvents) && queue_.hasNewEvent(); ++n)
    {
        auto event = queue_.popEvent();
        assert(event);
        processEvent(*event, localMap);
    }

    int64_t updateEnd = utils::system_time_us();

#ifdef DEBUG_TIMING
    if(haveDataToTransmit_)
    {
        std::cout << "INFO:GlobalTopoDirector:Update complete:" << ((updateEnd - updateStart) / 1000) << "ms\n";
    }
#endif

   transmitCalculatedOutput(communicator);

    // global_topo_hssh should always be running
    return system::UpdateStatus::running;
}


void GlobalTopoDirector::shutdown(system::ModuleCommunicator& communicator)
{
    // TODO: Save the topological map here?
}


// Data handlers
void GlobalTopoDirector::handleData(const LocalAreaEventVec& events, const std::string& channel)
{
    queue_.handleData(events, channel);
}


void GlobalTopoDirector::handleData(const GlobalTopoCommandPtr& command, const std::string& channel)
{
    queue_.handleData(command, channel);
}


void GlobalTopoDirector::handleData(const LocalPose& pose, const std::string& channel)
{
    queue_.handleData(pose, channel);
}


void GlobalTopoDirector::handleData(const LocalTopoMap& localMap, const std::string& channel)
{
    queue_.handleData(localMap, channel);
}


void GlobalTopoDirector::processCommand(const GlobalTopoCommand& command)
{
    command.print(std::cout);
    command.issue(*slam_);
}


void GlobalTopoDirector::processEvent(const LocalAreaEvent& event, const LocalTopoMap& localMap)
{
    slam_->addEvent(event, localMap);
    slam_->estimateTopologicalState();
    haveDataToTransmit_ = true;
}


void GlobalTopoDirector::transmitCalculatedOutput(system::ModuleCommunicator& communicator)
{
    if(!haveDataToTransmit_)
    {
        return;
    }

    haveDataToTransmit_ = false;

    auto bestMap = slam_->bestEstimate();
    communicator.sendMessage(bestMap);

    system::DebugCommunicator debug(communicator);
    slam_->sendDebug(debug);
}

} // namespace hssh
} // namespace vulcan
