/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     utils.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for localization:
 *
 *   - next_path_event : search for the next path event that occurred given the current event count
 *   - apply_turn_around_events : finds the topological direction after all path events are applied
 */

#include "hssh/global_topological/localization/utils.h"
#include "hssh/global_topological/state.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

TopologicalVisit::PathEventIter next_path_event(const TopologicalState& state, const TopologicalVisit& visit)
{
    // If the visit doesn't correspond to the current state, then no event could have occurred
    if (state.visitDepth != visit.depth()) {
        return visit.endPathEvents();
    }

    // If event count starts at 0, then it is the entry event.
    int eventCount = state.visitEventCount - 1;

    // if entry hasn't been processed, then the loop below will run all the through because eventCount == -1, which
    // is the desired behavior.
    for (auto eventIt = visit.beginPathEvents(), endIt = visit.endPathEvents(); eventIt != endIt; ++eventIt) {
        // If the event count hits, 0, then this is the desired event
        if (eventCount == 0) {
            return eventIt;
        }

        --eventCount;
    }

    // If the event count hasn't hit 0 before exiting, then all path events have been considered
    return visit.endPathEvents();
}


TopoDirection apply_turn_around_events(TopoDirection initial,
                                       TopologicalVisit::PathEventIter beginEvent,
                                       TopologicalVisit::PathEventIter endEvent)
{
    TopoDirection direction = initial;

    for (auto& event : boost::make_iterator_range(beginEvent, endEvent)) {
        // the current direction can never switch to being null
        assert(event.currentDirection() != TopoDirection::null);

        // If a direction hasn't been assigned yet, then just assign the current direction
        if (direction == TopoDirection::null) {
            direction = event.currentDirection();
        }
        // Otherwise, the direction has just flipped to the opposite, which is determined by the localization, so
        // don't worry about what the event actually says
        else {
            direction = opposite_direction(direction);
        }
    }

    return direction;
}

}   // namespace hssh
}   // namespace vulcan
