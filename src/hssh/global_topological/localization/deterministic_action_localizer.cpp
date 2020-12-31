/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     deterministic_action_localizer.cpp
 * \author   Collin Johnson
 *
 * Definition of DeterministicActionLocalizer.
 */

#include "hssh/global_topological/localization/deterministic_action_localizer.h"
#include "core/pose.h"
#include "hssh/global_topological/localization/utils.h"
#include "hssh/global_topological/state.h"
#include "hssh/global_topological/utils/visit.h"
#include <cassert>
#include <iostream>

#define DEBUG_PLACE_EVENT
#define DEBUG_PATH_EVENT

namespace vulcan
{
namespace hssh
{

DeterministicActionLocalizer::DeterministicActionLocalizer(void)
{
}


GlobalLocationDistribution DeterministicActionLocalizer::localize(const TopologicalState& state,
                                                                  const TopologicalVisit& visit)
{
    // If this is a new visit, then it must be that a new area was entered.
    if (state.visitDepth != visit.depth()) {
        handleEnteredEvent(state, visit);
    }
    // Otherwise, need to get the corresponding event, which is either a path event
    else if (next_path_event(state, visit) != visit.endPathEvents()) {

    }
    // or the exited event
    else if (visit.exitEvent()) {
    }

    return GlobalLocationDistribution();
}


GlobalLocationDistribution DeterministicActionLocalizer::handleEnteredEvent(const TopologicalState& state,
                                                                            const TopologicalVisit& visit)
{
}


GlobalLocationDistribution DeterministicActionLocalizer::handleExitedEvent(const TopologicalState& state,
                                                                           const TopologicalVisit& visit)
{
}


GlobalLocationDistribution DeterministicActionLocalizer::handleTurnAroundEvents(const TopologicalState& state,
                                                                                const TopologicalVisit& visit)
{
}


void DeterministicActionLocalizer::localize(const TopoMapPtr& map, const topology_action_t& action)
{
    if (action.haveEntered) {
        handleEnteredPlaceEvent(action.enteredAction, map);
    }

    if (action.haveExited) {
        handleExitedPlaceEvent(action.exitedAction, map);
    }

    if (action.haveReverse) {
        handlePathEvent(action.reverseAction, map);
    }
}


void DeterministicActionLocalizer::handleEnteredPlaceEvent(const entered_place_action_t& event, const TopoMapPtr& map)
{
#ifdef DEBUG_PLACE_EVENT
    std::cout << "DEBUG:DeterministicActionLocalizer:Entered a new place\n";
#endif

    GlobalLocation state = map->getGlobalLocation();

    state.onPath = false;

    if (state.pathState.targetPlaceId == PLACE_FRONTIER_ID) {
        state.placeId = PLACE_FRONTIER_ID;

#ifdef DEBUG_PLACE_EVENT
        std::cout << "DEBUG:DeterministicActionLocalizer:Arrived at frontier place along path " << state.pathId
                  << " from place " << state.pathState.entryPlaceId << '\n';
#endif
    } else   // must be at some place
    {
        if (map->hasPlace(state.pathState.targetPlaceId)) {
            LargeScaleStar placeStar(event.topology, event.entryPath);
            GlobalPlace& targetPlace = map->getPlace(state.pathState.targetPlaceId);

            state = updateTargetPlaceState(state, placeStar, targetPlace);

#ifdef DEBUG_PLACE_EVENT
            std::cout << "DEBUG:DeterministicActionLocalizer:Arrived at known place " << state.pathState.targetPlaceId
                      << " from " << state.pathState.entryPlaceId << " along path " << state.pathId << '\n';
#endif
        } else {
            std::cerr << "ERROR:DeterministicActionLocalizer:Hypothesis map doesn't contain the target place:"
                      << state.pathState.targetPlaceId << std::endl;
            assert(false);
        }
    }

    map->setGlobalLocation(state);
}


void DeterministicActionLocalizer::handleExitedPlaceEvent(const exited_place_action_t& event, const TopoMapPtr& map)
{
    GlobalLocation state = map->getGlobalLocation();

    state.onPath = true;

    state.pathState.entryPlaceId = state.placeId;

    if (map->hasPlace(state.placeId)) {
        GlobalPlace& exitPlace = map->getPlace(state.placeId);

        global_path_fragment_t exitFragment =
          exitPlace.getStar().findExitFragment(event.topology, event.entryPath, event.exitPath);

        if (map->hasPath(exitFragment.pathId)) {
            const GlobalPath& currentPath = map->getPath(exitFragment.pathId);
            GlobalPathSegment segment =
              currentPath.nextSegmentAlongPath(path_transition_t(exitPlace.getId(), exitFragment.fragmentId),
                                               exitFragment.direction);

            state.pathDirection = exitFragment.direction;
            state.pathId = exitFragment.pathId;

            state.pathState.entryFragmentId = exitFragment.fragmentId;
            state.pathState.targetPlaceId = (exitFragment.direction == PATH_MINUS)
              ? segment.getMinusTransition().placeId
              : segment.getPlusTransition().placeId;
            state.pathState.segmentIndex = currentPath.getSegmentIndex(segment);

            assert(state.pathState.targetPlaceId != PATH_ENDPOINT_ID);

#ifdef DEBUG_PLACE_EVENT
            std::cout << "DEBUG:DeterministicActionLocalizer:Exited place " << state.pathState.entryPlaceId << ':'
                      << state.pathState.entryFragmentId << " onto path " << state.pathId << "->" << state.pathDirection
                      << " with target " << state.pathState.targetPlaceId << '\n';
#endif
        } else {
            std::cerr << "ERROR:DeterministicActionLocalizer:Failed to find path " << exitFragment.pathId
                      << " associated with exit segment " << exitFragment.fragmentId << std::endl;
            assert(false);
        }
    } else {
        std::cerr << "ERROR:DeterministicActionLocalizer: Failed to find prior place in the map:" << state.placeId
                  << std::endl;
        assert(false);
    }

    map->setGlobalLocation(state);
}


void DeterministicActionLocalizer::handlePathEvent(const reverse_path_direction_action_t& event, const TopoMapPtr& map)
{
    // When reversing the path direction, ensure that the location is actually on a path. After that, the change is easy
    // and just needs to reverse the path direction and flip the target and entry places for the path

    GlobalLocation state = map->getGlobalLocation();

    if (state.onPath) {
        std::swap(state.pathState.entryPlaceId, state.pathState.targetPlaceId);
        state.pathDirection = opposite_direction(state.pathDirection);

#ifdef DEBUG_PATH_EVENT
        std::cout << "DEBUG:DeterministicActionLocalizer: Reversing path direction. New target:"
                  << state.pathState.targetPlaceId << '\n';
#endif

        map->setGlobalLocation(state);
    } else {
        std::cerr << "ERROR:DeterministicActionLocalizer: Tried to apply reverse path direction while not on a path"
                  << std::endl;
        assert(state.onPath);
    }
}


GlobalLocation DeterministicActionLocalizer::updateTargetPlaceState(const GlobalLocation& state,
                                                                    const LargeScaleStar& enteredPlaceStar,
                                                                    GlobalPlace& targetPlace)
{
    GlobalLocation updatedState = state;

    const LargeScaleStar& targetStar = targetPlace.getStar();

    updatedState.placeId = targetPlace.getId();

    // Now find the entry segment based on the star
    const std::vector<global_path_fragment_t>& segments = targetStar.getPathFragments();

    // When entering a place from a path, the direction of the segment of entry will be opposite the motion along the
    // path
    path_direction_t segmentDirection = opposite_direction(state.pathDirection);

    for (auto segmentIt = segments.begin(), segmentEnd = segments.end(); segmentIt != segmentEnd; ++segmentIt) {
        if ((segmentIt->pathId == updatedState.pathId) && (segmentIt->direction == segmentDirection)) {
            targetPlace.setEntryFragment(*segmentIt);
            updatedState.placeState.entryFragmentId = segmentIt->fragmentId;

            break;
        }
    }

#ifdef DEBUG_PLACE_EVENT
    std::cout << "DEBUG:DeterministicActionLocalizer:Arrived at expected place " << targetPlace.getId()
              << " along path " << updatedState.pathId << " through segment " << updatedState.placeState.entryFragmentId
              << '\n';
#endif

    // If the place and target don't match, just issue a warning because the non-match will cause the place to be pruned
    // by the mapper
    if (!targetStar.areStarsCompatible(enteredPlaceStar)) {
        assert(!enteredPlaceStar.areStarsCompatible(targetStar));
        updatedState.placeId = PLACE_INVALID_ID;
        std::cerr
          << "WARNING:DeterministicActionLocalizer:Failed to match entered place topology with expected topology.\n";
    }

    return updatedState;
}

}   // namespace hssh
}   // namespace vulcan
