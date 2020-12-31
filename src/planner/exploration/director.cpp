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
* Definition of ExplorationDirector.
*/

#include "planner/exploration/director.h"
#include "planner/exploration/map_explorer.h"
#include <unistd.h>

namespace vulcan
{
namespace planner
{

ExplorationDirector::ExplorationDirector(std::unique_ptr<MapExplorer> explorer)
: initialized_(false)
, explorer_(std::move(explorer))
{
}


ExplorationDirector::~ExplorationDirector(void)
{
    // For std::unique_ptr
}


void ExplorationDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    explorer_->subscribeToData(communicator);
}


system::TriggerStatus ExplorationDirector::waitForTrigger(void)
{
    // If data is available, then ready for the initial update
    if(explorer_->hasNewData())
    {
        return system::TriggerStatus::ready;
    }

    // Otherwise, wait a bit before returning so the module doesn't eat all the CPU idling
    usleep(10000);

    return system::TriggerStatus::not_ready;
}


system::UpdateStatus ExplorationDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    if(!initialized_)
    {
        explorer_->startExploring(communicator);
        initialized_ = true;
    }

    explorer_->continueExploring(communicator);

    // If finished exploring, then the exploration module can be shut down.
    if(explorer_->isFinishedExploring())
    {
        return system::UpdateStatus::finished;
    }
    else
    {
        return system::UpdateStatus::running;
    }
}


void ExplorationDirector::shutdown(system::ModuleCommunicator& communicator)
{
    explorer_->stopExploring(communicator);
}

} // namespace planner
} // namespace vulcan
