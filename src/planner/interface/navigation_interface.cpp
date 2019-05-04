/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface.cpp
* \author   Collin Johnson
* 
* Definition of NavigationInterface.
*/

#include <planner/interface/navigation_interface.h>
#include <planner/interface/decision_action.h>
#include <planner/interface/goal_action.h>
#include <mpepc/metric_planner/messages.h>
#include <system/system_communicator.h>
#include <fstream>

namespace vulcan
{
namespace planner
{

NavigationInterface::NavigationInterface(std::string moduleName)
: moduleName_(std::move(moduleName))
, goalIsDeferred_(false)
{
}


NavigationInterface::~NavigationInterface(void)
{
    // For std::unique_ptr
}


bool NavigationInterface::hasFinishedAction(const mpepc::metric_planner_status_message_t& status)
{
    // Order to evaluate is to check if there's a decision action, if yes, see if it is done
    // When an action completes, clear it out of the
    if(decisionAction_)
    {
        if(decisionAction_->isComplete(status))
        {
            decisionAction_.reset();
            return true;
        }
        else
        {
            return false;
        }
    }

    // Otherwise, if there's a goal action, check if it has finished
    if(goalAction_)
    {
        if(goalAction_->isComplete(status))
        {
            goalAction_.reset();
            return true;
        }
        else
        {
            return false;
        }
    }

    if(status.status == mpepc::SUCCESS_REACHED_POSE)
    {
        std::cout << "WARNING: NavigationInterface: Reached a pose that wasn't commanded by NavigationInterface.\n";
    }

    // If there are no actions, the robot must have finished whatever it was commanded to do
    return true;
}


void NavigationInterface::goToDecision(const Decision& decision,
                                       const pose_t& robotPose,
                                       system::SystemCommunicator& communicator)
{
    decisionAction_.reset(new DecisionAction(decision, robotPose));
    decisionAction_->perform(communicator);

    // If a goal was being performed, it is now deferred
    if(goalAction_)
    {
        goalIsDeferred_ = true;

        std::cout << "INFO: NavigationInterface: Deferring goal: " << goalAction_->goal().name() << '\n';
    }
}


int NavigationInterface::determineDecisions(const hssh::LocalTopoMap& map,
                                            const hssh::LocalLocation& location,
                                            const pose_t& pose)
{
    return decisionInterface_.determineActions(map, location, pose);
}


void NavigationInterface::goToGoal(const Goal& goal,
                                   const hssh::LocalTopoMap* topoMap,
                                   system::SystemCommunicator& communicator)
{
    // Issuing a new goal clears any existing decision or goal commands
    decisionAction_.reset();

    if(topoMap && goal.localArea())
    {
        goalAction_.reset(new GoalAction(goal, *topoMap));
    }
    else
    {
        goalAction_.reset(new GoalAction(goal));
    }
    goalAction_->perform(communicator);
    goalIsDeferred_ = false;
}


bool NavigationInterface::goToGoal(const std::string& goalName,
                                   const hssh::LocalTopoMap* topoMap,
                                   system::SystemCommunicator& communicator)
{
    auto goalIt = std::find_if(goalInterface_.begin(), goalInterface_.end(), [&goalName](const Goal& g) {
        return g.name() == goalName;
    });

    if(goalIt != goalInterface_.end())
    {
        goToGoal(*goalIt, topoMap, communicator);
    }

    return goalIt != goalInterface_.end();
}


boost::optional<Goal> NavigationInterface::currentGoal(void) const
{
    if(goalAction_)
    {
        return goalAction_->goal();
    }
    else
    {
        return boost::none;
    }
}


bool NavigationInterface::hasDeferredGoal(void) const
{
    return goalIsDeferred_;
}


void NavigationInterface::resumeDeferredGoal(system::SystemCommunicator& communicator)
{
    assert(goalAction_);
    assert(goalIsDeferred_);

    goalAction_->perform(communicator);
    // Once resumed, the goal is no longer deferred
    goalIsDeferred_ = false;
}


void NavigationInterface::addGoalGlobalPose(const std::string& name, const pose_t& pose)
{
    goalInterface_.addGlobalPose(name, pose);
}


void NavigationInterface::addGoalLocalArea(const std::string& name, hssh::LocalArea::Id id)
{
    goalInterface_.addLocalArea(name, id);
}


bool NavigationInterface::saveGoals(const std::string& filename)
{
    std::ofstream out(filename);

    bool success = false;

    if(out.is_open())
    {
        success = goalInterface_.save(out);
    }

    if(!success)
    {
        std::cerr << "ERROR: NavigationInterface: Failed to load goals from " << filename << '\n';
    }

    return success;
}


bool NavigationInterface::loadGoals(const std::string& filename)
{
    std::ifstream in(filename);

    std::size_t numLoaded = 0;

    if(in.is_open())
    {
        numLoaded = goalInterface_.load(in);
    }

    if(numLoaded == 0)
    {
        std::cerr << "ERROR: NavigationInterface: Failed to load goals from " << filename << '\n';
    }

    return numLoaded > 0;
}

} // namespace planner
} // namespace vulcan
