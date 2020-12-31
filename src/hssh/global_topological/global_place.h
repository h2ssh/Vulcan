/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_place.h
* \author   Collin Johnson
*
* Definition of GlobalPlace.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PLACE_H
#define HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PLACE_H

#include "hssh/global_topological/area.h"
#include "hssh/global_topological/transition_cycle.h"
#include "hssh/utils/id.h"
#include "core/pose.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{

/**
* GlobalPlace is the fundamental abstraction of the topological map. A place represents a decision point in
* the environment. A GlobalPlace is represented symbolically as a GlobalTransitionCycle. The GlobalPlace contains a GlobalPath
* for each path segment in the large-scale cycle_. A GlobalPlace corresponds to a node in the graph.
*/
class GlobalPlace
{
public:

    using Ptr = std::shared_ptr<GlobalPlace>;

    /**
    * Default constructor for GlobalPlace.
    */
    GlobalPlace(void) = default;

    /**
    * Constructor for GlobalPlace.
    *
    * \param    id              Id to assign to the place
    * \param    type            Type of place (either destination or decision point)
    * \param    metricId        Id of the local metric place abstracted by the global topo place
    * \param    cycle           GlobalTransitionCycle that symbolically represents the place
    */
    GlobalPlace(Id id,
                AreaType type,
                Id metricId,
                const GlobalTransitionCycle& cycle);

    /**
    * id retrieves the id of the place.
    */
    Id id(void) const { return id_; }

    /**
    * type retrieves the type of the place -- either decision point or destination.
    */
    AreaType type(void) const { return type_; }

    /**
    * toArea retrieves the GlobalArea description of the place.
    */
    GlobalArea toArea(void) const { return GlobalArea(id_, type_); }

    /**
    * cycle retrieves the GlobalTransitionCycle associated with the place.
    */
    const GlobalTransitionCycle& cycle(void) const { return cycle_; }

    /**
    * metricPlaceId retrieves the id of the local metric place associated with this global topo place.
    */
    Id metricPlaceId(void) const { return metricPlaceIds_.empty() ? kInvalidId : metricPlaceIds_.front(); }

    /**
    * replaceTransition replaces a transition stored in the area. The transition is being replaced because some sort
    * of loop closure or area has been added such that the area on the other side has changed in some way.
    *
    * \param    oldTrans            Transition currently contained in the path segment
    * \param    newTrans            New transition to use for the path segment
    * \return   True if oldTrans was found and successfully replaced with newTrans.
    */
    bool replaceTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans);

    /**
    * changeMetricId changes one of the stored metric place ids. The metric id usually changes between when a robot
    * enters and exits an area. Generally, the view of the environment at exit is more accurate than at entry, so
    * the exit version of the metric place should be saved.
    *
    * \param    oldId           Id to replace
    * \param    newId           New id to use
    * \return   True if oldId was found and replaced with newId.
    */
    bool changeMetricId(Id oldId, Id newId);

private:

    Id id_ = kInvalidId;
    AreaType type_ = AreaType::place;
    GlobalTransitionCycle cycle_;
    std::vector<Id> metricPlaceIds_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id_,
            type_,
            cycle_,
            metricPlaceIds_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PLACE_H
