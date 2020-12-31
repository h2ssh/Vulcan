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
 * Declaration of ExplorationDirector.
 */

#ifndef PLANNER_EXPLORATION_DIRECTOR_H
#define PLANNER_EXPLORATION_DIRECTOR_H

#include "system/director.h"
#include "utils/condition_variable.h"
#include <memory>

namespace vulcan
{
namespace planner
{

class MapExplorer;


/**
 * ExplorationDirector orchestrates the actions taken in the map_exploration module. The director makes use of the
 * provided MapExplorer in the following way:
 *
 *   - subscribeToData:
 *       - explorer->subscribeToData
 *   - waitForTrigger:
 *       - explorer->hasNewData
 *   - runUpdate:
 *       - explorer->startExploring (on initialization)
 *       - explorer->continueExploring (during normal updates)
 *       - explorer->isFinishedExploring (check if finished, if so, exploration ends and module exits)
 *   - shutdown:
 *       - explorer->stopExploring (clean up on exploration completion)
 */
class ExplorationDirector : public system::Director
{
public:
    /**
     * Constructor for ExplorationDirector.
     */
    ExplorationDirector(std::unique_ptr<MapExplorer> explorer);

    /**
     * Destructor for ExplorationDirector.
     */
    ~ExplorationDirector(void);

    // Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

private:
    bool initialized_;
    std::unique_ptr<MapExplorer> explorer_;

    utils::ConditionVariable dataTrigger_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_EXPLORATION_DIRECTOR_H
