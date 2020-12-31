/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_explorer.cpp
* \author   Collin Johnson
* 
* Definition of LocalTopoExplorer.
*/

#include "planner/exploration/local_topo/local_topo_explorer.h"
#include "planner/exploration/local_topo/exploration_status.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/events/turn_around.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/stub.h"
#include "utils/timestamp.h"
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace planner
{

void print_area(hssh::LocalArea::Id id, const hssh::LocalTopoMap& map);


///////////////// local_topo_explorer_params_t ///////////////////
local_topo_explorer_params_t::local_topo_explorer_params_t(const utils::ConfigFile& config)
{
    // No parameters currently needed
}


LocalTopoExplorer::LocalTopoExplorer(const hssh::LocalTopoMap& map, const local_topo_explorer_params_t& params)
: state_(waiting_for_initialization)
, map_(map)
, explorationMap_(map)
, params_(params)
, haveNewData_(false)
, haveExplorationUpdate_(false)
{
}


LocalTopoExplorer::~LocalTopoExplorer(void)
{
    // For std::unique_ptr
}


void LocalTopoExplorer::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<hssh::LocalPose>(this);
    communicator.subscribeTo<hssh::LocalLocation>(this);
    communicator.subscribeTo<hssh::LocalAreaEventVec>(this);
    communicator.subscribeTo<mpepc::metric_planner_status_message_t>(this);
}


void LocalTopoExplorer::unsubscribeFromData(system::ModuleCommunicator& communicator) 
{
    PRINT_STUB("LocalTopoExplorer::unsubscribeFromData");
}


bool LocalTopoExplorer::hasNewData(void)
{
    // Exploration needs to wakeup and check whenever any new data arrives because either a status message arrived or
    // some timeout needs to fire
    return haveNewData_;
}


bool LocalTopoExplorer::isFinishedExploring(void) const
{
    return state_ == finished_exploring;
}


void LocalTopoExplorer::startExploring(system::ModuleCommunicator& communicator)
{
    // To start exploring, first select a random area to go to first.
    LocalAreaTarget* start = explorationMap_.selectRandomTarget();
    
    if(start)
    {
        goToTarget(start, communicator);
        
        std::cout << "INFO: LocalTopoExplorer: Driving to starting area:";
        print_area(start->areaId(), map_);
        std::cout << '\n';
        startPose_ = pose_;
        currentTarget_ = start;
        state_ = driving_to_initial_area;
    }
    // There are targets to explore, so finish immediately
    else
    {
        assert(explorationMap_.sizeUnvisited() == 0);
        
        std::cerr << "WARNING: LocalTopoExplorer: Map contained no unvisited areas to explore. Num total areas:"
            << explorationMap_.sizeVisited();
        state_ = finished_exploring;
    }
}


void LocalTopoExplorer::continueExploring(system::ModuleCommunicator& communicator)
{
    utils::AutoMutex autoLock(dataLock_);
    
    switch(state_)
    {
    case waiting_for_initialization:
        // Nothing to do
        break;
        
    case driving_to_initial_area:
        driveToInitialArea(communicator);
        break;
        
    case exploring_map:
        exploreMap(communicator);
        break;

    case driving_to_start:
        driveToStart(communicator);
        break;
        
    case finished_exploring:
    default:
        // Nothing needs to be done once finished
        break;
    }
    
    // Clear out all stored data needed for updates
    events_.clear();
    taskStatus_.clear();
    haveNewData_ = false;

    if(haveExplorationUpdate_)
    {
        local_topo_exploration_status_t status;
        status.explorationMap = explorationMap_;
        status.currentArea = location_.areaId();
        status.targetArea = currentTarget_ ? currentTarget_->areaId() : -1;
        status.plannerTask = currentPlannerTask_;

        communicator.sendMessage(status);

        haveExplorationUpdate_ = false;
    }
}


void LocalTopoExplorer::stopExploring(system::ModuleCommunicator& communicator)
{
    // Once exploration stops, then cancel any remaining tasks.
    mpepc::metric_planner_command_message_t command;
    command.timestamp = utils::system_time_us();
    command.command = mpepc::CANCEL;
    
    communicator.sendMessage(command);
}


void LocalTopoExplorer::handleData(const hssh::LocalPose& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    pose_ = pose;
    
    haveNewData_ = true;
}


void LocalTopoExplorer::handleData(const hssh::LocalLocation& location, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    if(location_.areaId() != location.areaId())
    {
        haveExplorationUpdate_ = true;
    }

    location_ = location;
    
    haveNewData_ = true;
}


void LocalTopoExplorer::handleData(const hssh::LocalAreaEventVec& events, const std::string& channel)
{
    // Add the new events to the end of the previous events, as the previous events might not have been processed
    utils::AutoMutex autoLock(dataLock_);
    events_.insert(events_.end(), events.begin(), events.end());

    // Whenever events occur, some sort of exploration event has occurred
    haveExplorationUpdate_ = true;
    
    haveNewData_ = true;
}


void LocalTopoExplorer::handleData(const mpepc::metric_planner_status_message_t& status, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    taskStatus_.push_back(status);
    
    haveNewData_ = true;
}


void LocalTopoExplorer::goToTarget(LocalTopoExplorationTarget* target, system::ModuleCommunicator& communicator)
{
    currentPlannerTask_ = target->explorationTask(pose_.pose());
    communicator.sendMessage(std::static_pointer_cast<mpepc::MetricPlannerTask>(currentPlannerTask_));

    haveExplorationUpdate_ = true;
}


bool LocalTopoExplorer::haveReachedCurrentTarget(void)
{
    // The current target is reached once the MPEPC status says that its task is completed.
    bool isMpepcTaskFinished = false;
    
    for(auto& status : taskStatus_)
    {
        isMpepcTaskFinished |= status.status == mpepc::SUCCESS_REACHED_POSE;
    }
    
    // If the task is finished, but haven't reached the desired LocalArea, then raise a warning, but continue with life
    if(isMpepcTaskFinished && currentTarget_ && (currentTarget_->areaId() != location_.areaId()))
    {
        std::cerr << "WARNING: LocalTopoExplorer: Finished MPEPC task, but was not in the expected area. Actual:";
        print_area(location_.areaId(), map_);
        std::cout << " Expected:";
        print_area(currentTarget_->areaId(), map_);
        std::cout << '\n';
    }
    
    return isMpepcTaskFinished;
}


LocalTopoExplorer::PlannerTaskStatus LocalTopoExplorer::checkPlannerTaskStatus(void)
{
    if(taskStatus_.empty() || !currentPlannerTask_)
    {
        return PlannerTaskStatus::unknown;
    }
    
    for(auto& status : taskStatus_)
    {
        if(status.taskId == currentPlannerTask_->id())
        {
            if((status.status == mpepc::ACTIVE_NORMAL) || (status.status == mpepc::PAUSED))
            {
                return PlannerTaskStatus::executing;
            }
        }
    }
    
    return PlannerTaskStatus::not_executing;
}


void LocalTopoExplorer::driveToInitialArea(system::ModuleCommunicator& communicator)
{
    assert(state_ == driving_to_initial_area);
    
    // If at the target, then clear out the current target and switch to exploring mode
    if(haveReachedCurrentTarget())
    {
        // When switching to exploration, set the first target to maintain invariant for that state
        currentTarget_ = explorationMap_.selectRandomTarget();
        state_ = exploring_map;

        if(currentTarget_)
        {
            goToTarget(currentTarget_, communicator);
            std::cout << "INFO: LocalTopoExplorer: Selected new target:";
            print_area(currentTarget_->areaId(), map_);
            std::cout << '\n';
        }
    }
    // If the planner isn't current executing the task, it failed to be received, so give it another try
    else if(currentTarget_ && (checkPlannerTaskStatus() == PlannerTaskStatus::not_executing))
    {
        goToTarget(currentTarget_, communicator);
    }
}


void LocalTopoExplorer::exploreMap(system::ModuleCommunicator& communicator)
{
    assert(state_ == exploring_map);
    assert(currentTarget_); // exploring always has an active target
    
    // Process any newly visited areas
    int numVisited = explorationMap_.identifyVisitedTargets(events_);
    
    if(numVisited > 0)
    {
        std::cout << "INFO: LocalTopoExplorer: Visited " << numVisited << " new areas.\n";
    }
    
    // If the current target has been reached, then either:
    //  - select a new target
    //  - switch to finished exploration because no unvisited targets remain
    if(currentTarget_ && haveReachedCurrentTarget())
    {
        std::cout << "INFO: LocalTopoExplorer: Reached target:";
        print_area(currentTarget_->areaId(), map_);
        std::cout << '\n';

        currentTarget_ = explorationMap_.selectRandomTarget();
        state_ = currentTarget_ ? exploring_map : driving_to_start;
        
        if(currentTarget_)
        {
            goToTarget(currentTarget_, communicator);

            std::cout << "INFO: LocalTopoExplorer: Selected new target:";
            print_area(currentTarget_->areaId(), map_);
            std::cout << '\n';
        }
        // Drive to the start
        else
        {
            currentPlannerTask_ = std::make_shared<mpepc::NavigationTask>(startPose_.pose());
            communicator.sendMessage(std::static_pointer_cast<mpepc::MetricPlannerTask>(currentPlannerTask_));

            haveExplorationUpdate_ = true;
        }
    }
    // If the planner isn't current executing the task, it failed to be received, so give it another try
    else if(currentTarget_ && (checkPlannerTaskStatus() == PlannerTaskStatus::not_executing))
    {
        std::cout << "INFO: LocalTopoExplorer: metric_planner failed to start executing task. Resending.\n";
        goToTarget(currentTarget_, communicator);
    }
}


void LocalTopoExplorer::driveToStart(system::ModuleCommunicator& communicator)
{
    if(haveReachedCurrentTarget())
    {
        std::cout << "INFO: LocalTopoExplorer: Reached starting pose. Finished exploring.\n";
        state_ = planner::LocalTopoExplorer::finished_exploring;
    }
    // If the planner isn't current executing the task, it failed to be received, so give it another try
    else if(checkPlannerTaskStatus() != PlannerTaskStatus::executing)
    {
        std::cout << "INFO: LocalTopoExplorer: metric_planner failed to start executing task to drive to start. Resending.\n";
        currentPlannerTask_ = std::make_shared<mpepc::NavigationTask>(startPose_.pose());
        communicator.sendMessage(std::static_pointer_cast<mpepc::MetricPlannerTask>(currentPlannerTask_));

        haveExplorationUpdate_ = true;
    }
}


void print_area(hssh::LocalArea::Id id, const hssh::LocalTopoMap& map)
{
    auto areaPtr = map.areaWithId(id);

    if(areaPtr)
    {
        std::cout << "Area " << id << ':' << areaPtr->boundary();
    }
}

} // namespace planner
} // namespace vulcan
