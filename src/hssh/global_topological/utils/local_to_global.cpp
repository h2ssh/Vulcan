/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_to_global.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions that convert from the local topological representation of an area to a global
 * topological representation of the area:
 *
 *   - find_place_exit_transition
 *   - find_relative_place_exit
 */

#include "hssh/global_topological/utils/local_to_global.h"
#include "hssh/global_topological/global_path_segment.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/global_topological/utils/visit.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/areas/place.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/local_topo_map.h"
#include "utils/cyclic_iterator.h"
#include "utils/stub.h"
#include <algorithm>

// #define DEBUG_PLACE_EXIT

namespace vulcan
{
namespace hssh
{

// Where did the robot leave the path?
enum class PathTransitionType
{
    plus,
    minus,
    same,
    opposite,
    sequence,
    none,
};


GlobalTransition
  entry_transition(const LocalPlace& localPlace, const Gateway& entryGateway, const GlobalPlace& globalPlace);
GlobalTransition entry_transition(const LocalPathSegment& localSegment,
                                  const Gateway& entryGateway,
                                  const GlobalPathSegment& globalSegment);
boost::optional<GlobalTransition> entry_along_sequence(const std::vector<TransitionAffordance>& localSequence,
                                                       const Gateway& entryGateway,
                                                       const GlobalTransitionSequence& globalSequence);

int place_exit_index(const LocalArea& area, const Gateway& entry, const Gateway& exit);
int absolute_transition_index(const LocalArea& area, const Gateway& transition);
PathTransitionType
  local_path_exit_type(const LocalArea& area, const boost::optional<Gateway>& entry, const Gateway& exit);


std::pair<GlobalPlace, GlobalTransition> create_global_place_from_local_place(const GlobalArea& globalArea,
                                                                              boost::optional<Gateway> entryGateway,
                                                                              const LocalArea& localArea)
{
    // Fail to create a place if the pased in area isn't a place
    if (!is_place_type(localArea.type())) {
        return std::pair<GlobalPlace, GlobalTransition>();
    }

    const LocalPlace* localPlace = static_cast<const LocalPlace*>(&localArea);

    GlobalTransitionCycle cycle(globalArea, localPlace->star());
    GlobalPlace globalPlace(globalArea.id(), localArea.type(), localArea.id(), cycle);

    // If there is an entry gateway, then find the corresponding entry transition
    if (entryGateway) {
        return std::make_pair(globalPlace, entry_transition(*localPlace, *entryGateway, globalPlace));
    }
    // Otherwise, this is the initial area, so we don't need to do anything.
    else {
        return std::make_pair(globalPlace, GlobalTransition());
    }
}


std::pair<GlobalPathSegment, GlobalTransition>
  create_global_path_segment_from_local_path_segment(const GlobalArea& globalArea,
                                                     boost::optional<Gateway> entryGateway,
                                                     const LocalArea& localArea,
                                                     bool isExplored)
{
    // Fail to create a path segment if the pased in area isn't a path segment
    if (localArea.type() != AreaType::path_segment) {
        return std::pair<GlobalPathSegment, GlobalTransition>();
    }

    const LocalPathSegment* localSegment = static_cast<const LocalPathSegment*>(&localArea);

    Line<float> center(localSegment->minusTransition().gateway().center(),
                       localSegment->plusTransition().gateway().center());
    GlobalTransitionSequence left(globalArea, localSegment->leftDestinations(), center);
    GlobalTransitionSequence right(globalArea, localSegment->rightDestinations(), center);
    GlobalTransition minus(next_id(),
                           globalArea,
                           GlobalArea(localSegment->minusTransition().type()),
                           NavigationStatus::navigable,
                           ExplorationStatus::frontier);
    GlobalTransition plus(next_id(),
                          globalArea,
                          GlobalArea(localSegment->plusTransition().type()),
                          NavigationStatus::navigable,
                          ExplorationStatus::frontier);

    auto lambda = localSegment->lambda();

    GlobalPathSegment globalSegment(globalArea.id(), plus, minus, lambda, left, right, isExplored);
    // If there is an entry gateway, then find the corresponding entry transition
    if (entryGateway) {
        return std::make_pair(globalSegment, entry_transition(*localSegment, *entryGateway, globalSegment));
    }
    // Otherwise, this is the initial area, so we don't need to do anything.
    else {
        return std::make_pair(globalSegment, GlobalTransition());
    }
}


GlobalTransition find_place_exit_transition(const GlobalTransitionCycle& cycle,
                                            const GlobalTransition& entry,
                                            const TopologicalVisit& visit)
{
    auto entryIt = std::find(cycle.begin(), cycle.end(), entry);

    int relativeExit = find_relative_place_exit_index(visit);

    // If we have a valid entry and the relative exit is possible within the cycle, then find it
    if ((entryIt != cycle.end()) && (relativeExit >= 0) && (relativeExit < static_cast<int>(cycle.size()))) {
        auto cycIt = utils::cyclic_iterator<GlobalTransitionCycle::Iter>(entryIt, cycle.begin(), cycle.end());
        std::advance(cycIt, relativeExit);
        return *cycIt;
    }

#ifdef DEBUG_PLACE_EXIT
    std::cerr
      << "WARNING: find_place_exit_transition: No relative exit transition was found, so only the absolute index "
      << "of the exit transition is used. It might be unreliable...Relative exit: " << relativeExit
      << " cycle: " << cycle.size() << "\n";
#endif   // DEBUG_PLACE_EXIT

    auto exit = visit.exitEvent();
    if (exit && exit->transitionGateway()) {
        int exitIndex = absolute_transition_index(*exit->exitedArea(), *exit->transitionGateway());
        // If there's a valid exit, then take it
        if ((exitIndex >= 0) && (exitIndex < static_cast<int>(cycle.size()))) {
            return cycle[exitIndex];
        }
    }

    // Otherwise, the exit transition doesn't exist, so an invalid exit must be returned. Most likely localizing
    // in an invalid map hypothesis.
    return GlobalTransition();
}


int find_relative_place_exit_index(const TopologicalVisit& visit)
{
    auto entry = visit.entryEvent();
    auto exit = visit.exitEvent();

    // If no exit event has occurred yet, then there isn't a valid index
    if (!exit || !entry.transitionGateway() || !exit->transitionGateway()) {
        return -1;
    }

    // Find the relative index for both the entry and exit events to see that they match. They might not. If they don't,
    // defer to the exit index, as it contains more complete knowledge of the area.
    int entryIndex = -1;
    int exitIndex = -1;

    if (entry.enteredArea()) {
        entryIndex = place_exit_index(*entry.enteredArea(), *entry.transitionGateway(), *exit->transitionGateway());
    }

    if (exit->exitedArea()) {
        exitIndex = place_exit_index(*exit->exitedArea(), *entry.transitionGateway(), *exit->transitionGateway());
    }

#ifdef DEBUG_PLACE_EXIT
    if (entryIndex != exitIndex) {
        std::cerr << "WARNING: find_relative_place_exit_index: Didn't find the same relative index: Via entry:"
                  << entryIndex << " Via exit:" << exitIndex << '\n';
    }
#endif   // DEBUG_PLACE_EXIT

    return (exitIndex != -1) ? exitIndex : entryIndex;
}


GlobalTransition find_path_exit_transition(const GlobalPathSegment& pathSegment,
                                           const GlobalTransition& entry,
                                           const TopologicalVisit& visit)
{
    // If there is no exit event, then there can't be an exit transition
    if (!visit.exitEvent()) {
        return GlobalTransition();
    }

    auto exitType = local_path_exit_type(*visit.exitEvent()->exitedArea(),
                                         visit.entryEvent().transitionGateway(),
                                         *visit.exitEvent()->transitionGateway());

    switch (exitType) {
    case PathTransitionType::plus:
        return pathSegment.plusTransition();

    case PathTransitionType::minus:
        return pathSegment.minusTransition();

    case PathTransitionType::opposite:
        return (pathSegment.plusTransition() == entry) ? pathSegment.minusTransition() : pathSegment.plusTransition();

    case PathTransitionType::same:
        return entry;

    case PathTransitionType::sequence:
        std::cerr << "WARNING: Exiting to a sequence not yet supported!\n";
        break;

    case PathTransitionType::none:
        std::cerr << "ERROR: find_path_exit_transition: Found no exit transition.\n";
        break;
    }

    return GlobalTransition();
}


bool path_endpoints_were_swapped(const GlobalPathSegment& entryPathSegment,
                                 const GlobalTransition& entryTransition,
                                 const GlobalPathSegment& exitPathSegment,
                                 const GlobalTransition& exitTransition,
                                 const TopologicalVisit& visit)
{
    return false;

    //     const LocalPathSegment* path = static_cast<const LocalPathSegment*>(visit.localArea());
    //
    //     // If no entry gateway, started at a path segment, so need to just assign based on which transition in local
    //     path
    //     // segment matches. This approach is fine because global path segment direction is same as local direction
    //     for
    //     // the initial area.
    //     if(!visit.entryEvent().transitionGateway())
    //     {
    //         return false;
    //     }
    //
    //     // If the entry and exit transitions are not the same, but are at the plus/minus ends, then exited through
    //     the
    //     // opposite transition
    //     if((path->plusTransition().gateway().isSimilarTo(*entry) &&
    //     path->minusTransition().gateway().isSimilarTo(exit))
    //         || (path->minusTransition().gateway().isSimilarTo(*entry) &&
    //         path->plusTransition().gateway().isSimilarTo(exit)))
    //     {
    //         return PathTransitionType::opposite;
    //     }
    //     // If the entry and exit transitions are the same gateway, then the robot exited through the same transition
    //     else if((path->plusTransition().gateway().isSimilarTo(*entry) &&
    //     path->plusTransition().gateway().isSimilarTo(exit))
    //         || (path->minusTransition().gateway().isSimilarTo(*entry) &&
    //         path->minusTransition().gateway().isSimilarTo(exit)))
    //     {
    //         return PathTransitionType::same;
    //     }
    //     // Otherwise, must've exited along the sequence somewhere
    //     else
    //     {
    //         return PathTransitionType::sequence;
    //     }
}


GlobalTransition
  entry_transition(const LocalPlace& localPlace, const Gateway& entryGateway, const GlobalPlace& globalPlace)
{
    // Find the gateway fragment in the local place. This index is the offset into the global transition cycle.
    auto entryFrag = localPlace.findGatewayFragment(entryGateway);

    // If an entry was found, then the index can be used to get the GlobalTransition
    if (entryFrag) {
        auto& cycle = globalPlace.cycle();
        return cycle[entryFrag->fragmentId];
    }
    // Otherwise, no transition exists
    std::cerr << "ERROR: Found no entry transition for a newly created place!\n";
    return GlobalTransition();
}


GlobalTransition entry_transition(const LocalPathSegment& localSegment,
                                  const Gateway& entryGateway,
                                  const GlobalPathSegment& globalSegment)
{
    // The entry is either one of the endpoints or along one of the sequences
    if (localSegment.minusTransition().gateway().isSimilarTo(entryGateway)) {
        return globalSegment.minusTransition();
    }

    if (localSegment.plusTransition().gateway().isSimilarTo(entryGateway)) {
        return globalSegment.plusTransition();
    }

    auto leftEntry = entry_along_sequence(localSegment.leftDestinations(), entryGateway, globalSegment.leftSequence());
    if (leftEntry) {
        return *leftEntry;
    }

    auto rightEntry =
      entry_along_sequence(localSegment.rightDestinations(), entryGateway, globalSegment.rightSequence());
    if (rightEntry) {
        return *rightEntry;
    }

    std::cerr << "ERROR: Found no entry transition for a newly created path segment! Going with closest end\n";
    // Selecting the closest endpoint transition
    double minusDist =
      distance_between_points(localSegment.minusTransition().gateway().center(), entryGateway.center());
    double plusDist = distance_between_points(localSegment.plusTransition().gateway().center(), entryGateway.center());
    return plusDist < minusDist ? globalSegment.plusTransition() : globalSegment.minusTransition();
}


boost::optional<GlobalTransition> entry_along_sequence(const std::vector<TransitionAffordance>& localSequence,
                                                       const Gateway& entryGateway,
                                                       const GlobalTransitionSequence& globalSequence)
{
    // Iterate through the sequence. If a matching gateway is found, then it matches up with the index of the global
    // sequence

    for (std::size_t n = 0; n < localSequence.size(); ++n) {
        if (localSequence[n].gateway().isSimilarTo(entryGateway)) {
            // If an entry is found that's safe, take it
            if (n < globalSequence.size()) {
                return globalSequence[n];
            }
        }
    }

    // Didn't find a transition. So must be some sort of invalid state.
    return boost::none;
}


int place_exit_index(const LocalArea& area, const Gateway& entry, const Gateway& exit)
{
    // Must be a place
    if (!is_place_type(area.type())) {
        return -1;
    }

    const LocalPlace* place = static_cast<const LocalPlace*>(&area);
    auto entryFrag = place->findGatewayFragment(entry);
    auto exitFrag = place->findGatewayFragment(exit);

    if (entryFrag && exitFrag) {
        int entryIndex = entryFrag->fragmentId;
        int exitIndex = exitFrag->fragmentId;

        // Exit is after entry in the arbitrary fragment ordering
        if (entryIndex <= exitIndex) {
            return exitIndex - entryIndex;
        }
        // Wrap around to the end
        else {
            int numFragments = std::distance(place->star().begin(), place->star().end());
            return (numFragments - entryIndex) + exitIndex;
        }
    }
#ifdef DEBUG_PLACE_EXIT
    else {
        std::cerr << "place_exit_index: Found entry? " << static_cast<bool>(entryFrag) << " Found exit? "
                  << static_cast<bool>(exitFrag) << " Star: " << place->star() << '\n';
    }
#endif

    // One of the fragments couldn't be found, so there isn't an index
    return -1;
}


int absolute_transition_index(const LocalArea& area, const Gateway& transition)
{
    if (is_place_type(area.type())) {
        const LocalPlace* place = static_cast<const LocalPlace*>(&area);
        auto frag = place->findGatewayFragment(transition);
        return frag ? frag->fragmentId : -1;
    }

    return -1;
}


PathTransitionType
  local_path_exit_type(const LocalArea& area, const boost::optional<Gateway>& entry, const Gateway& exit)
{
    if (area.type() != AreaType::path_segment) {
        return PathTransitionType::none;
    }

    const LocalPathSegment* path = static_cast<const LocalPathSegment*>(&area);

    // If no entry gateway, started at a path segment, so need to just assign based on which transition in local path
    // segment matches. This approach is fine because global path segment direction is same as local direction for
    // the initial area.
    if (!entry) {
        if (path->plusTransition().gateway().isSimilarTo(exit)) {
            return PathTransitionType::plus;
        } else if (path->minusTransition().gateway().isSimilarTo(exit)) {
            return PathTransitionType::minus;
        } else   // exited along a sequence
        {
            return PathTransitionType::sequence;
        }
    }

    // If the entry and exit transitions are not the same, but are at the plus/minus ends, then exited through the
    // opposite transition
    if ((path->plusTransition().gateway().isSimilarTo(*entry) && path->minusTransition().gateway().isSimilarTo(exit))
        || (path->minusTransition().gateway().isSimilarTo(*entry)
            && path->plusTransition().gateway().isSimilarTo(exit))) {
        return PathTransitionType::opposite;
    }
    // If the entry and exit transitions are the same gateway, then the robot exited through the same transition
    else if ((path->plusTransition().gateway().isSimilarTo(*entry)
              && path->plusTransition().gateway().isSimilarTo(exit))
             || (path->minusTransition().gateway().isSimilarTo(*entry)
                 && path->minusTransition().gateway().isSimilarTo(exit))) {
        return PathTransitionType::same;
    }
    // Otherwise, must've exited along the sequence somewhere
    else {
        return PathTransitionType::sequence;
    }
}

}   // namespace hssh
}   // namespace vulcan
