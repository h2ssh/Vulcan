/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_route.h
* \author   Collin Johnson
*
* Definition of GoalRoute.
*/

#ifndef PLANNER_GOAL_GOAL_ROUTE_H
#define PLANNER_GOAL_GOAL_ROUTE_H

#include <hssh/global_topological/global_place.h>
#include <hssh/global_topological/global_path_segment.h>
#include <vector>

namespace vulcan
{
namespace planner
{

/**
* goal_route_element_t defines the different entities that can be part of a GoalRoute.
*/
struct goal_route_element_t
{
    bool isPathSegment;
    bool isPlace;

    hssh::GlobalPathSegment segment;
    hssh::GlobalPlace       place;

    goal_route_element_t(void)
    {
    }

    goal_route_element_t(const hssh::GlobalPathSegment& segment)
        : isPathSegment(true)
        , isPlace(false)
        , segment(segment)
    {
    }

    goal_route_element_t(const hssh::GlobalPlace& place)
        : isPathSegment(false)
        , isPlace(true)
        , place(place)
    {
    }
};

/**
* GoalRoute defines a plan through the global topological layer of the HSSH. A plan
* consists of an alternating sequence of Places and PathSegments. Collectively, the Places
* and PathSegments are goal_route_element_t. The element is essentially a union with
* a flag indicating which type it actually contains.
*
* A GoalRoute supports a confirmation flag to allow a person to be in the loop when a
* plan is being formulated. The GoalTarget for the plan specifies if it needs to be
* confirmed. An unconfirmed plan will not generate a DecisionTargetSequence for further
* execution. Once confirmed, the local topo planner will begin executing the plan.
*/
class GoalRoute
{
public:

    /**
    * Default constructor for GoalRoute.
    */
    GoalRoute(void) { }

    /**
    * Constructor for GoalRoute.
    *
    * \param    id              Unique identifier for the plan to provide an easy means of referring to a specific plan
    * \param    sequence        The individual elements that make up the plan
    * \param    confirmed       Flag indicating if the plan has been confirmed (optional, default = true)
    */
    GoalRoute(uint32_t id, const std::vector<goal_route_element_t>& sequence, bool confirmed = true)
        : id(id)
        , sequence(sequence)
        , confirmed(confirmed)
    {
    }

    /**
    * getId retrieves the unique ID for this plan.
    */
    uint32_t getId(void) const { return id; }

    /**
    * getSequence retrieves the sequence of interleaved GlobalPlaces and GlobalPathSegments that make up the plan.
    */
    const std::vector<goal_route_element_t>& getSequence(void) const { return sequence; }

    /**
    * isConfirmed checks to see if this plan has been confirmed.
    */
    bool isConfirmed(void) const { return confirmed; }

private:

    uint32_t id;
    std::vector<goal_route_element_t> sequence;
    bool confirmed;
};

}
}

#endif // PLANNER_GOAL_GOAL_ROUTE_H
