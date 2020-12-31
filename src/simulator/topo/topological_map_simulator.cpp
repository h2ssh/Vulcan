/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     topological_map_simulator.cpp
 * \author   Collin Johnson
 *
 * Definition of TopologicalMapSimulator.
 */

#include "simulator/topo/topological_map_simulator.h"
#include "hssh/global_topological/utils/metric_map_cache.h"
#include "planner/decision/decision_target.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace simulator
{

TopologicalMapSimulator::TopologicalMapSimulator(const topological_map_simulator_params_t& params) : params(params)
{
}

void TopologicalMapSimulator::setMapToSimulate(const hssh::TopologicalMap& map, const hssh::GlobalLocation& state)
{
    topoMap = map;
    mapState = state;
}


void TopologicalMapSimulator::setMapState(const hssh::GlobalLocation& state)
{
    mapState = state;
}


event_type_t TopologicalMapSimulator::simulateTarget(const std::shared_ptr<planner::DecisionTarget>& target)
{
    switch (target->getType()) {
    case planner::LOCAL_TOPO_PLACE_NEIGHBORHOOD:
        return simulatePlaceNeighborhoodTarget(target);

    case planner::LOCAL_TOPO_RELATIVE_PLACE:
        return simulateRelativePlaceTarget(target);

    case planner::LOCAL_TOPO_LOCAL_PATH:
        return simulateLocalPathTarget(target);
    }

    return NO_EVENT;
}


event_type_t
  TopologicalMapSimulator::simulatePlaceNeighborhoodTarget(const std::shared_ptr<planner::DecisionTarget>& target)
{
    assert(!mapState.onPath);

    std::shared_ptr<planner::PlaceNeighborhoodTarget> placeTarget =
      std::static_pointer_cast<planner::PlaceNeighborhoodTarget>(target);

    return LOCAL_PLACE_EVENT;
}


event_type_t
  TopologicalMapSimulator::simulateRelativePlaceTarget(const std::shared_ptr<planner::DecisionTarget>& target)
{
    assert(!mapState.onPath);

    std::shared_ptr<planner::RelativePlaceTarget> placeTarget =
      std::static_pointer_cast<planner::RelativePlaceTarget>(target);

    int8_t exitOffset = 0;

    switch (placeTarget->getCommand()) {
    case planner::LOCAL_TOPO_TURN_LEFT:
        exitOffset = -1;
        break;

    case planner::LOCAL_TOPO_TURN_RIGHT:
        exitOffset = 1;
        break;

    case planner::LOCAL_TOPO_GO_STRAIGHT:
        exitOffset = 2;
        break;

    case planner::LOCAL_TOPO_GO_BACK:
    default:
        exitOffset = 0;
        break;
    }

    int8_t fragmentId = mapState.placeState.entryFragmentId + exitOffset;

    const hssh::LocalPlace& local = placeManager.load();

    return LOCAL_PLACE_EVENT;
}


event_type_t TopologicalMapSimulator::simulateLocalPathTarget(const std::shared_ptr<planner::DecisionTarget>& target)
{
    assert(mapState.onPath);

    std::shared_ptr<planner::LocalPathTarget> pathTarget = std::static_pointer_cast<planner::LocalPathTarget>(target);

    return LOCAL_PLACE_EVENT;
}

}   // namespace simulator
}   // namespace vulcan
