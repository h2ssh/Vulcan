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
 * Definition of GoalDirector.
 */

#include "planner/goal/director.h"
#include "planner/goal/goal_progress.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include <cassert>
#include <iostream>

// #define DEBUG_STATE_MACHINE

namespace vulcan
{
namespace planner
{

GoalDirector::GoalDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: params(load_goal_params(config))
, state(WAITING_FOR_DATA)
, haveMap(false)
, haveNewTarget(false)
, havePlanConfirmation(false)
, havePlanCancellation(false)
, haveNewRoute(false)
, haveNewSequence(false)
, haveNewProgress(false)
, haveNewDebugInfo(false)
{
}


void GoalDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
}


system::TriggerStatus GoalDirector::waitForTrigger(void)
{
    return !dataTrigger.timedWait(100) : system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus GoalDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    processAvailableData();
    transmitCalculatedOutput(communicator);

    // The goal_planner is always running
    return system::UpdateStatus::running;
}


void GoalDirector::shutdown(system::ModuleCommunicator& communicator)
{
    // TODO: Halt the operation of the remaining planners if something is in progress
}


void GoalDirector::handleData(const hssh::TopologicalMap& topoMap, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    map = topoMap;
    haveMap = true;

    dataTrigger.setPredicate(true);
    dataTrigger.broadcast();
}


void GoalDirector::handleData(const std::vector<hssh::TopologicalMap>& maps, const std::string& channel)
{
    // Ignore for now, as only the best map is being used
}


void GoalDirector::handleData(const GoalTarget& target, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    this->target = target;
    haveNewTarget = true;

    dataTrigger.setPredicate(true);
    dataTrigger.broadcast();
}


void GoalDirector::handleData(const goal_route_command_message_t& message, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    routeMessages.push_back(message);

    // or-equal here because multiple messages could arrive before the messages are processed, in which case
    // the cancellation or confirmation could be clobbered
    havePlanCancellation |= message.command == CANCEL_ROUTE;
    havePlanConfirmation |= message.command == CONFIRM_ROUTE;

    dataTrigger.setPredicate(true);
    dataTrigger.broadcast();
}


void GoalDirector::handleData(const DecisionProgress& progress, const std::string& channel)
{
    // TODO: Do I actually need the DecisionProgress?
}


void GoalDirector::processAvailableData(void)
{
    utils::AutoMutex autoLock(dataLock);

    // Run the state machine until the state doesn't change, meaning all new data has been processed
    while (runStateMachine())
        ;

    dataTrigger.setPredicate(false);
}


void GoalDirector::transmitCalculatedOutput(system::ModuleCommunicator& communicator)
{
    if (haveNewRoute) {
    }

    if (haveNewProgress) {
    }

    if (haveNewDebugInfo) {
    }

    if (haveNewSequence) {
    }

    haveNewRoute = false;
    haveNewSequence = false;
    haveNewProgress = false;
    haveNewDebugInfo = false;
}


bool GoalDirector::runStateMachine(void)
{
    director_state_t startingState = state;

    switch (state) {
    case WAITING_FOR_DATA:
        waitingForData();
        break;

    case CALCULATING_PLAN:
        calculatePlan();
        break;

    case WAITING_FOR_CONFIRMATION:
        waitingForConfirmation();
        break;

    case EXECUTING_PLAN:
        monitorPlan();
        break;

    default:
        std::cerr << "ERROR: GoalDirector: Unknown director state!\n";
        assert(false);
    }

#ifdef DEBUG_STATE_MACHINE
    std::cout << "DEBUG:Goal: Start state:" << startingState << " End state:" << state << '\n';
#endif

    return startingState != state;
}


void GoalDirector::waitingForData(void)
{
    if (haveDataForNewPlan()) {
        state = CALCULATING_PLAN;
    }
}


bool GoalDirector::haveDataForNewPlan(void)
{
    return haveMap && haveNewTarget;
}


void GoalDirector::calculatePlan(void)
{
    haveNewRoute = planner.plan(target, map, debug);

    if (target.shouldConfirm()) {
        state = WAITING_FOR_CONFIRMATION;
    } else {
        monitor.setRouteToMonitor(planner.getRoute());

        state = EXECUTING_PLAN;
        haveNewSequence = true;
    }

    haveNewDebugInfo = true;
    haveNewTarget = false;
}


void GoalDirector::waitingForConfirmation(void)
{
    if (havePlanConfirmation && isPlanConfirmed()) {
        monitor.setRouteToMonitor(planner.getRoute());

        state = EXECUTING_PLAN;
        haveNewSequence = true;
    } else if (isPlanCancelled()) {
        state = WAITING_FOR_DATA;
    }
}


bool GoalDirector::isPlanConfirmed(void)
{
    return haveMessageWithProperties(planner.getRoute().getId(), CONFIRM_ROUTE);
}


bool GoalDirector::isPlanCancelled(void)
{
    return haveMessageWithProperties(planner.getRoute().getId(), CANCEL_ROUTE);
}


void GoalDirector::monitorPlan(void)
{
    monitor.updateProgress(map.getGlobalLocation());

    if (needNewPlan()) {
        state = WAITING_FOR_DATA;
    }

    haveNewProgress = true;
}


bool GoalDirector::needNewPlan(void)
{
    return monitor.finishedRoute() || haveNewTarget || isPlanCancelled();
}


bool GoalDirector::haveMessageWithProperties(uint32_t planId, route_command_t command)
{
    for (auto messageIt = routeMessages.begin(), messageEnd = routeMessages.end(); messageIt != messageEnd;
         ++messageIt) {
        if ((messageIt->planId == planId) && (messageIt->command == command)) {
            if (command == CONFIRM_ROUTE) {
                std::cout << "INFO:GoalPlanner: Plan " << messageIt->planId << " confirmed via " << messageIt->source
                          << '\n';
            } else if (command == CANCEL_ROUTE) {
                std::cout << "INFO:GoalPlanner: Plan " << messageIt->planId << " cancelled via " << messageIt->source
                          << '\n';
            }

            return true;
        }
    }

    return false;
}

}   // namespace planner
}   // namespace vulcan
