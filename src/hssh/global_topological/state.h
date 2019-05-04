/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_state.h
* \author   Collin Johnson
* 
* Declaration of TopologicalState.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_STATE_H
#define HSSH_GLOBAL_TOPOLOGICAL_STATE_H

#include <hssh/global_topological/chi.h>
#include <hssh/global_topological/global_location.h>
#include <hssh/global_topological/map_probability.h>
#include <hssh/global_topological/topological_map.h>
#include <hssh/utils/id.h>
#include <system/message_traits.h>
#include <cereal/types/memory.hpp>

namespace vulcan
{
namespace hssh
{

/**
* TopologicalState represents a possible topological state of the robot. The state contains:
* 
*   - a unique identifier to be used for identifying the state in the TreeOfMaps
*   - a topological map representing the robot's knowledge of the environment
*   - the probability distribution for the map
*   - the location of the robot within this topological map
*   - the depth of the current visit being handled
*   - the number of events incorporated from the current visit
*/
struct TopologicalState
{
    Id id = kInvalidId;
    TopologicalMap::Ptr map;
    TopoMapProbability probability;
    GlobalLocation location;
    bool needsOptimization = false;
    int visitDepth = -1;
    int numPlaceVisits = 0;
    int visitEventCount = 0;
    
    // Serialization support
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id,
            map,
            probability,
            location,
            visitDepth,
            numPlaceVisits,
            visitEventCount
        );
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::TopologicalState, ("GLOBAL_TOPO_STATE"));

#endif // HSSH_GLOBAL_TOPOLOGICAL_STATE_H
