/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     localizer.cpp
 * \author   Collin Johnson
 *
 * Definition of TopologicalLocalizer.
 */

#include "hssh/global_topological/localization/localizer.h"
#include "hssh/global_topological/global_path_segment.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/global_topological/localization/utils.h"
#include "hssh/global_topological/state.h"
#include "hssh/global_topological/utils/local_to_global.h"
#include "utils/algorithm_ext.h"
#include <cassert>
#include <iostream>

// #define DEBUG_ENTERED
// #define DEBUG_EXITED
// #define DEBUG_TURNED
// #define DEBUG_COMPATIBILITY

namespace vulcan
{
namespace hssh
{

GlobalLocationDistribution find_location_from_previous_exit(const GlobalArea& prevArea,
                                                            const GlobalTransition& exitTrans,
                                                            const TopologicalMap& map);


GlobalLocationDistribution TopologicalLocalizer::localize(const TopologicalState& state, const TopologicalVisit& visit)
{
    // No events have been considered? Then entering the area
    if (state.visitEventCount == 0) {
        // Just check in debug mode if the entered event is indeed valid.
        assert(verifyEnteredEvent(state, visit));
    }

    auto newLocation = state.location;
    if (state.location.areaType == AreaType::path_segment) {
        newLocation.pathDirection = handleTurnAroundEvents(state, next_path_event(state, visit), visit.endPathEvents());
    }

    return handleExitedEvent(state, newLocation, visit);
}


bool TopologicalLocalizer::verifyEnteredEvent(const TopologicalState& state, const TopologicalVisit& visit)
{
#ifdef DEBUG_ENTERED
    std::cout << "DEBUG: TopologicalLocalizer: Entered:" << visit.depth() << " Expected:" << state.visitDepth << '\n';
#endif   // DEBUG_ENTERED

    // The depth of the visit should be same as stored depth because we want to process everything up to the exit.
    return state.visitDepth == visit.depth();
}


GlobalLocationDistribution TopologicalLocalizer::handleExitedEvent(const TopologicalState& state,
                                                                   const GlobalLocation& location,
                                                                   const TopologicalVisit& visit)
{
    auto exit = visit.exitEvent();

    if (!exit) {
        std::cerr << "ERROR: TopologicalLocalizer: Trying to handle exit, but no exit event exists.\n";
        return GlobalLocationDistribution(location);
    }

    if (location.areaType == AreaType::path_segment) {
        return handlePathExit(state, location, visit);
    } else if (is_place_type(location.areaType)) {
        return handlePlaceExit(state, location, visit);
    }

    return GlobalLocationDistribution();
}


TopoDirection TopologicalLocalizer::handleTurnAroundEvents(const TopologicalState& state,
                                                           TopologicalVisit::PathEventIter beginEvent,
                                                           TopologicalVisit::PathEventIter endEvent)
{
    TopoDirection finalDirection = apply_turn_around_events(state.location.pathDirection, beginEvent, endEvent);

#ifdef DEBUG_TURNED
    std::cout << "DEBUG: TopologicalLocalizer: Turned around " << std::distance(beginEvent, endEvent) << " times."
              << " Start dir: " << state.location.pathDirection << " Final dir: " << finalDirection << '\n';
#endif   // DEBUG_TURNED

    return finalDirection;
}


GlobalLocationDistribution TopologicalLocalizer::handlePathExit(const TopologicalState& state,
                                                                const GlobalLocation& location,
                                                                const TopologicalVisit& visit)
{
    // Retrieve the current path segment
    // Find the exit transition
    // Determine which place is on the other side of exit, which will be the final location
    auto path = state.map->getPathSegment(location.areaId);
    assert(path);   // we can't be nowhere!

    auto exitTrans = find_path_exit_transition(*path, location.entryTransition, visit);

    // If the exit is invalid, then we aren't anywhere!
    if (exitTrans.id() == kInvalidId) {
        std::cerr << "ERROR: TopologicalLocalizer: Could not find an exit from path " << location.areaId << '\n';
        return GlobalLocationDistribution();
    }

    return find_location_from_previous_exit(path->toArea(), exitTrans, *state.map);
}


GlobalLocationDistribution TopologicalLocalizer::handlePlaceExit(const TopologicalState& state,
                                                                 const GlobalLocation& location,
                                                                 const TopologicalVisit& visit)
{
    // Retrieve the current place the robot is in
    // Find the exit transition
    // Determine the area on the other side of the transition, which must be the new location of the robot
    auto place = state.map->getPlace(location.areaId);
    assert(place);   // the place must exist or we have an ill-formed state and that's a programming error!
    auto exitTrans = find_place_exit_transition(place->cycle(), location.entryTransition, visit);

    // Construct the view of the entry to the area via the visit to confirm that the cycles are compatible. In the
    // case of arriving via a frontier path, the compatibility isn't actually checked, so need to fix that problem.
    auto newPlace =
      create_global_place_from_local_place(GlobalArea(), visit.entryEvent().transitionGateway(), *visit.localArea());
    bool areCompatible =
      are_cycles_compatible(place->cycle(), location.entryTransition, newPlace.first.cycle(), newPlace.second);
    if (!areCompatible) {
#ifdef DEBUG_COMPATIBILITY
        std::cerr << "DEBUG: TopologicalLocalizer: Threw away place after getting there because it wasn't actually "
                     "compatible.\n"
                  << " From map: (" << place->cycle() << " entry: " << location.entryTransition << " From visit: ("
                  << newPlace.first.cycle() << " entry: " << newPlace.second << '\n';
#endif   // DEBUG_COMPATIBILITY
        return GlobalLocationDistribution();
    }

    // If the exit transition is invalid, then we aren't anywhere! We found an invalid hypothesis
    if (exitTrans.id() == kInvalidId) {
        return GlobalLocationDistribution();
    }

    return find_location_from_previous_exit(place->toArea(), exitTrans, *state.map);
}


GlobalLocationDistribution find_location_from_previous_exit(const GlobalArea& prevArea,
                                                            const GlobalTransition& exitTrans,
                                                            const TopologicalMap& map)
{
    auto nextArea = exitTrans.otherArea(prevArea);

    // If the next area isn't valid, then we aren't anywhere!
    if (nextArea.id() == kInvalidId) {
        return GlobalLocationDistribution();
    }

    // Otherwise, we have arrived at a new location.
    // If at a path segment, then need to construct the appropriate path location
    if (nextArea.type() == AreaType::path_segment) {
        // If we entered a frontier path segment, then we don't know where we are or what direction we are facing
        // This condition will only occur if we enter a corridor from a destination. Otherwise, the path segment
        // will always be known.
        if (nextArea.id() == kFrontierId) {
#ifdef DEBUG_EXITED
            std::cout << "DEBUG: TopologicalLocalizer: Entered frontier path segment.\n";
#endif   // DEBUG_EXITED
            return GlobalLocationDistribution(GlobalLocation(kFrontierId, exitTrans, TopoDirection::null));
        }

        const GlobalPathSegment* segment = map.getPathSegment(nextArea.id());

        // Ended up somewhere that we shouldn't/can't be, which means we're trying to navigate an invalid
        // map hypothesis. Fail by saying that we're nowhere.
        if (!segment) {
            std::cerr << "ERROR::Localizer: Can't find the path segment " << nextArea.id()
                      << ". Map must be invalid.\n";
            return GlobalLocationDistribution();
        }

        auto location = segment->locationOnSegment(exitTrans);

#ifdef DEBUG_EXITED
        std::cout << "DEBUG: TopologicalLocalizer: Entered path segment " << location.areaId
                  << " travelling in direction " << location.pathDirection << '\n';
#endif   // DEBUG_EXITED

        return GlobalLocationDistribution(location);
    }
    // Otherwise, the generic area location constructor can be used
    else {
#ifdef DEBUG_EXITED
        std::cout << "DEBUG: TopologicalLocalizer: Entered place " << nextArea.id() << '\n';
#endif   // DEBUG_EXITED

        return GlobalLocationDistribution(GlobalLocation(nextArea, exitTrans));
    }
}

}   // namespace hssh
}   // namespace vulcan
