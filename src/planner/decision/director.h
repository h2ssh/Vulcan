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
* Declaration of DecisionPlannerDirector.
*/

#ifndef PLANNER_DECISION_DIRECTOR_H
#define PLANNER_DECISION_DIRECTOR_H

#include <vector>
#include "system/director.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"
#include "planner/decision/planner.h"
#include "planner/decision/state.h"
#include "planner/decision/params.h"

namespace vulcan
{
namespace hssh { class LocalTopoMap;  }
namespace hssh { class LocalLocation; }
namespace planner
{

class DecisionCommand;

/**
* DecisionPlannerDirector coordinates data processing for the DecisionPlanner. It
* consumes the input, processes it, and sends output in the form of DecisionPlans.
*/
class DecisionPlannerDirector : public system::Director
{
public:

    /**
    * Constructor for DecisionPlannerDirector.
    *
    * \param    params          Parameters for the module
    */
    DecisionPlannerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const DecisionCommand&     command,  const std::string& channel);
    void handleData(const hssh::LocalTopoMap&  map,      const std::string& channel);
    void handleData(const hssh::LocalLocation& location, const std::string& channel);

private:

    utils::Mutex             dataLock;
    utils::ConditionVariable dataTrigger;
};

}
}

#endif // PLANNER_DECISION_DIRECTOR_H
