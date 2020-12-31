/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_exploration_mapper.cpp
* \author   Collin Johnson
*
* Definition of TreeExplorationMapper.
*/

#include <cassert>
#include <iostream>
#include "hssh/global_topological/actions.h"
#include "hssh/global_topological/measurements.h"
#include "hssh/global_topological/topological_map_hypothesis.h"
#include "hssh/global_topological/tree_of_maps.h"
#include "hssh/global_topological/mapping/tree_exploration_mapper.h"

#define DEBUG_PLACE_ENTRY

namespace vulcan
{
namespace hssh
{

TreeExplorationMapper::TreeExplorationMapper(MetricMapCache& manager)
    : TopologicalMapper(manager)
{
}


TreeExplorationMapper::~TreeExplorationMapper(void)
{
}


void TreeExplorationMapper::updateMap(const topology_action_t&       action,
                                      const topology_measurements_t& measurements)
{
    if(action.haveEntered)
    {
        const std::set<boost::shared_ptr<TopologicalMapHypothesis>>& leaves = tree.getLeaves();

        for(auto hypIt = leaves.begin(), hypEnd = leaves.end(); hypIt != hypEnd; ++hypIt)
        {
            handlePlaceEntered(action.enteredAction, measurements, *hypIt);
        }

        if(leaves.empty())
        {
            createInitialMap(action.enteredAction, measurements);
        }
    }
}


TopoMapPtr TreeExplorationMapper::getUsableHypothesis(void ) const
{
    const std::set<boost::shared_ptr<TopologicalMapHypothesis>>& leaves = tree.getLeaves();

    if(!leaves.empty())
    {
        return *(leaves.begin());
    }

    return TopoMapPtr();
}


void TreeExplorationMapper::handlePlaceEntered(const entered_place_action_t&               action,
                                               const topology_measurements_t&              measurements,
                                               boost::shared_ptr<TopologicalMapHypothesis> hypothesis)
{
    #ifdef DEBUG_PLACE_ENTRY
    std::cout<<"DEBUG:TreeExplorationMapper:Entered a new place\n";
    #endif

    // The state indicates if a new place has been entered if the placeId = PLACE_FRONTIER_ID
    GlobalLocation state = hypothesis->getGlobalLocation();

    if(state.placeId == PLACE_FRONTIER_ID)
    {
        LargeScaleStar largeStar(action.topology, action.entryPath);
        GlobalPlace    newPlace(PLACE_FRONTIER_ID, measurements.placeId, largeStar);

        boost::shared_ptr<TopologicalMapHypothesis> newHypothesis = tree.addNewPlace(hypothesis, newPlace, newPlace.getEntryFragment().fragmentId, measurements.lambda);

        state.placeId         = newPlace.getId();
        state.placeState.entryFragmentId = newPlace.getEntryFragment().fragmentId;

        newHypothesis->setGlobalLocation(state);

        #ifdef DEBUG_PLACE_ENTRY
        std::cout<<"DEBUG:TreeExplorationMapper:Created new place "<<state.placeId<<'\n';
        #endif
    }

    // Nothing to map if the place has been visited before
}


void TreeExplorationMapper::createInitialMap(const entered_place_action_t& action, const topology_measurements_t& measurements)
{
    LargeScaleStar largeStar(action.topology, action.entryPath);
    GlobalPlace newPlace(PLACE_FRONTIER_ID, measurements.placeId, largeStar);

    tree.createInitialHypothesis(newPlace);

    #ifdef DEBUG_PLACE_ENTRY
    std::cout<<"DEBUG:TreeExplorationMapper:Created initial map\n";
    #endif
}

} // namespace hssh
} // namespace vulcan
