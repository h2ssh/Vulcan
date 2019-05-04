/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_monitor.h
* \author   Collin Johnson
*
* Declaration of GoalMonitor.
*/

#ifndef PLANNER_GOAL_GOAL_MONITOR_H
#define PLANNER_GOAL_GOAL_MONITOR_H

#include <planner/goal/goal_route.h>
#include <vector>

namespace vulcan
{
namespace hssh { struct GlobalLocation; }

namespace planner
{

class GoalProgress;

/**
* GoalMonitor monitors the progress of the robot as it moves along its planned route.
* Each time an updated map arrived with a new robot state, it is checked against the expected
* position of the robot in the route. If the robot goes off its route, or the route cannot
* be navigated, i.e. path is blocked, an error is raised so replanning can happen.
*
* The progress is stored in a GoalProgress instance that indicates where the robot has been,
* where it is now, and where it is going.
*/
class GoalMonitor
{
public:

    /**
    * setRouteToMonitor sets the current route being traversed by the robot.
    */
    void setRouteToMonitor(const GoalRoute& route);

    /**
    * updateProgress updates the progress of the robot along the current route.
    *
    * \param    state           State of the robot within the current map
    */
    void updateProgress(const hssh::GlobalLocation& state);

    /**
    * finishedRoute checks to see if the robot has finished navigating the currently
    * executing route.
    */
    bool finishedRoute(void) const { return remaining.empty(); }

    /**
    * getRouteProgress retrieves the current progress of the robot moving along the route.
    */
    GoalProgress getRouteProgress(void) const;

private:

    void updatePathSegmentProgress(const hssh::GlobalLocation& state);
    void updatePlaceProgress      (const hssh::GlobalLocation& state);
    void activateNextRouteElement (void);

    GoalRoute route;

    std::vector<goal_route_element_t> visited;
    goal_route_element_t              active;
    std::vector<goal_route_element_t> remaining;
};

}
}

#endif // PLANNER_GOAL_GOAL_MONITOR_H
