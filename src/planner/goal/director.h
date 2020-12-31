/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.h
* \author   Collin Johnson
*
* Declaration of GoalDirector.
*/

#ifndef PLANNER_GOAL_DIRECTOR_H
#define PLANNER_GOAL_DIRECTOR_H

#include "hssh/global_topological/topological_map.h"
#include "planner/goal/params.h"
#include "planner/goal/goal_target.h"
#include "planner/goal/goal_planner.h"
#include "planner/goal/goal_monitor.h"
#include "planner/goal/messages.h"
#include "planner/goal/debug_info.h"
#include "planner/goal/consumers.h"
#include "utils/mutex.h"
#include "utils/condition_variable.h"
#include "system/director.h"

namespace vulcan
{
namespace planner
{

struct goal_params_t;

/**
* GoalDirector organizes the computation for the global topo planner. The global topo planner
* has three states:
*
*   - WAITING_FOR_DATA         : to execute a plan, both a map and a target are needed
*   - CALCULATING_PLAN         : when the necessary data have arrived, a new plan needs to be calculated
*   - WAITING_FOR_CONFIRMATION : some plans requires confirmation before they begin executing, here a plan has been created and waiting to start it
*   - EXECUTING_PLAN           : a plan is currently underway and being monitored for progress
*
* While waiting for data, the planner sits idle. When calculating a plan, the GoalPlanner takes
* over. When executing, the status of the plan is monitored with the GoalMonitor.
*/
class GoalDirector : public system::Director
{
public:

    /**
    * Constructor for GoalDirector.
    *
    * \param    params          Parameters for the module
    */
    GoalDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const hssh::TopologicalMap&              topoMap,  const std::string& channel);
    void handleData(const std::vector<hssh::TopologicalMap>& maps,     const std::string& channel);
    void handleData(const GoalTarget&                        target,   const std::string& channel);
    void handleData(const goal_route_command_message_t&      message,  const std::string& channel);
    void handleData(const DecisionProgress&                  progress, const std::string& channel);

private:

    enum director_state_t
    {
        WAITING_FOR_DATA,
        CALCULATING_PLAN,
        WAITING_FOR_CONFIRMATION,
        EXECUTING_PLAN
    };

    goal_params_t params;

    director_state_t state;

    GoalPlanner planner;
    GoalMonitor monitor;

    hssh::TopologicalMap                      map;
    GoalTarget                                target;
    std::vector<goal_route_command_message_t> routeMessages;
    goal_debug_info_t                         debug;

    bool haveMap;
    bool haveNewTarget;
    bool havePlanConfirmation;
    bool havePlanCancellation;

    bool haveNewRoute;
    bool haveNewSequence;
    bool haveNewProgress;
    bool haveNewDebugInfo;

    utils::Mutex             dataLock;
    utils::ConditionVariable dataTrigger;

    void processAvailableData(void);
    void transmitCalculatedOutput(system::ModuleCommunicator& communicator);

    // Return true if state changes, meaning state machine should run again, false if finished updating
    bool runStateMachine(void);

    // WAITING_FOR_DATA methods
    void waitingForData(void);
    bool haveDataForNewPlan(void);

    // CALCULATING_PLAN methods
    void calculatePlan(void);

    // WAITING_FOR_CONFIRMATION methods
    void waitingForConfirmation(void);
    bool isPlanConfirmed(void);
    bool isPlanCancelled(void);

    // EXECUTING_PLAN
    void monitorPlan(void);
    bool needNewPlan(void);

    // Helpers
    bool haveMessageWithProperties(uint32_t planId, route_command_t command);
};

}
}

#endif // PLANNER_GOAL_DIRECTOR_H
