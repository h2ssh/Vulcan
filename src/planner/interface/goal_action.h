/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal_action.h
 * \author   Collin Johnson
 *
 * Declaration of GoalAction.
 */

#ifndef PLANNER_INTERFACE_GOAL_ACTION_H
#define PLANNER_INTERFACE_GOAL_ACTION_H

#include "planner/interface/goal.h"
#include <memory>

namespace vulcan
{
namespace hssh
{
class LocalTopoMap;
}
namespace mpepc
{
class MetricPlannerTask;
}
namespace mpepc
{
struct metric_planner_status_message_t;
}
namespace system
{
class SystemCommunicator;
}
namespace planner
{

/**
 * GoalAction represents the action to be taken to reach a particular Goal. Action is executed via the perform method.
 *
 * Currently, the only action supported is navigation via the global metric map. Thus, the goal must have an associated
 * global metric pose in order to be executed.
 */
class GoalAction
{
public:
    /**
     * Constructor for GoalAction.
     *
     * \pre  goal has a globalMetricPose.
     * \param    goal            Goal to go to
     */
    explicit GoalAction(const Goal& goal);

    /**
     * Constructor for GoalAction.
     *
     * Create a GoalAction for driving to an area.
     *
     * \pre  goal has a localArea
     * \param    goal            Goal to go to
     * \param    topoMap         Map in which the robot is navigating
     */
    GoalAction(const Goal& goal, const hssh::LocalTopoMap& topoMap);

    /**
     * perform issues the action that will cause it to be executed.
     *
     * \param    communicator            Communicator to use for sending out the command
     */
    void perform(system::SystemCommunicator& communicator) const;

    /**
     * isComplete checks to see if the action has finished.
     *
     * \param    status          Status message from the planner indicating if the task has finished
     */
    bool isComplete(const mpepc::metric_planner_status_message_t& status) const;

    /**
     * goal retrieves the Goal to which this action is commanding the robot.
     */
    Goal goal(void) const { return goal_; }

private:
    Goal goal_;
    std::shared_ptr<mpepc::MetricPlannerTask> task_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_INTERFACE_GOAL_ACTION_H
