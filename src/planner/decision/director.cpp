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
 * Definition of DecisionPlannerDirector.
 */

#include "planner/decision/director.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace planner
{

DecisionPlannerDirector::DecisionPlannerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: dataTrigger(false)
{
}


void DecisionPlannerDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
}


system::TriggerStatus DecisionPlannerDirector::waitForTrigger(void)
{
    return !dataTrigger.timedWait(100) ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus DecisionPlannerDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    utils::AutoMutex autoLock(dataLock);

    dataTrigger.setPredicate(false);

    // The decision_planner is always running
    return system::UpdateStatus::running;
}


void DecisionPlannerDirector::shutdown(system::ModuleCommunicator& communicator)
{
}

}   // namespace planner
}   // namespace vulcan
