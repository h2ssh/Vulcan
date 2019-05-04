/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_progress.h
* \author   Collin Johnson
*
* Definition of GoalProgress.
*/

#ifndef PLANNER_GOAL_GOAL_PROGRESS_H
#define PLANNER_GOAL_GOAL_PROGRESS_H

#include <planner/goal/goal_route.h>
#include <vector>

namespace vulcan
{
namespace planner
{

/**
* GoalProgress defines progress metrics for the currently executing GoalRoute.
* The progress metrics indicate the currently executing portion of the plan, the parts of
* a plan already traversed, and those remaining. Additionally, error codes for the plan
* are provided.
*/
class GoalProgress
{
public:

    /**
    * Default constructor for GoalProgress.
    */
    GoalProgress(void) { }

    /**
    * Constructor for GoalProgress.
    *
    * \param    id              Id of the plan for which progress is being monitored
    * \param    visited         Sequence of elements visited so far
    * \param    active          Element currently being traversed
    * \param    remaining       Sequence of remaining elements to be visited
    */
    GoalProgress(uint32_t                                        id,
                       const std::vector<goal_route_element_t>& visited,
                       const goal_route_element_t&              active,
                       const std::vector<goal_route_element_t>& remaining)
        : id(id)
        , visited(visited)
        , active(active)
        , remaining(remaining)
    {
    }

    /**
    * getId retrieves the id of the plan for which this intsance is a progress metric.
    */
    uint32_t getId(void) const { return id; }

    /**
    * getVisitedElements retrieves the sequence of elements visited so far.
    */
    const std::vector<goal_route_element_t>& getVisitedElements(void) const { return visited; }

    /**
    * getActiveElement retrieves the element currently being traversed.
    */
    goal_route_element_t getActiveElement(void) const { return active; }

    /**
    * getRemainingElements retrieves the sequence of remaining elements in the plan.
    */
    const std::vector<goal_route_element_t>& getRemainingElements(void) const { return remaining; }

private:

    uint32_t id;
    std::vector<goal_route_element_t> visited;
    goal_route_element_t              active;
    std::vector<goal_route_element_t> remaining;
};

}
}

#endif // PLANNER_GOAL_GOAL_PROGRESS_H
