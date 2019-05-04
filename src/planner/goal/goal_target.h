/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_target.h
* \author   Collin Johnson
*
* Definition of GoalTarget.
*/

#ifndef PLANNER_GOAL_GOAL_TARGET_H
#define PLANNER_GOAL_GOAL_TARGET_H

#include <hssh/global_topological/global_place.h>
#include <cstdint>

namespace vulcan
{
namespace planner
{

/**
* GoalTarget defines a target to which a route should be planned. Currently, a target must be a GlobalPlace
* within a specified map. Once place hierarchies are in-place, the target could become substantially more complicated.
*/
class GoalTarget
{
public:

    /**
    * Default constructor for GoalTarget.
    */
    GoalTarget(void) { }

    /**
    * Constructor for GoalTarget.
    *
    * \param    id              Unique id assigned to the target
    * \param    mapId           Id of the map with which the target is associated
    * \param    target          Target place to move to
    * \param    shouldConfirm   Flag indicating if the route to the target needs confirmation (optional, default = false)
    */
    GoalTarget(uint32_t id, uint32_t mapId, const hssh::GlobalPlace& target, bool shouldConfirm = false)
        : id(id)
        , mapId(mapId)
        , target(target)
        , needsConfirmation(shouldConfirm)
    {
    }

    /**
    * getId retrieves the unique id assigned to this target.
    */
    uint32_t getId(void) const { return id; }

    /**
    * getMapId retrieves the id of the map with which this target is associated.
    */
    uint32_t getMapId(void) const { return mapId; }

    /**
    * getTargetPlace retrieves the target place to which a route should be planned.
    */
    const hssh::GlobalPlace getTargetPlace(void) const { return target; }

    /**
    * shouldConfirm indicates if the route to this target needs to be confirmed by the operator
    * before driving begins.
    */
    bool shouldConfirm(void) const { return needsConfirmation; }

private:

    uint32_t id;
    uint32_t mapId;
    hssh::GlobalPlace target;
    bool needsConfirmation;
};

}
}

#endif // PLANNER_GOAL_GOAL_TARGET_H
