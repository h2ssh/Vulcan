/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_monitor.cpp
 * \author   Collin Johnson
 *
 * Definition of GoalMonitor.
 */

#include "planner/goal/goal_monitor.h"
#include "hssh/global_topological/global_location.h"
#include "planner/goal/goal_progress.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace planner
{

void GoalMonitor::setRouteToMonitor(const GoalRoute& route)
{
    this->route = route;

    remaining = route.getSequence();
    activateNextRouteElement();
    visited.clear();   // make sure visited from previous route is cleared out
}


void GoalMonitor::updateProgress(const hssh::GlobalLocation& state)
{
    if (active.isPathSegment) {
        updatePathSegmentProgress(state);
    } else if (active.isPlace) {
        updatePlaceProgress(state);
    }
}


GoalProgress GoalMonitor::getRouteProgress(void) const
{
    return GoalProgress(route.getId(), visited, active, remaining);
}


void GoalMonitor::updatePathSegmentProgress(const hssh::GlobalLocation& state)
{
    /*
     * The active command has the robot on a path segment, and the valid path segment, presumably.
     * Now, if the state indicates the robot is no longer on a path, then it must have transitioned to the
     * next location on the route. Move to the next route element and then confirm that it has the correct
     * properties, ie. the correct placeId
     */
    if (!state.onPath) {
        activateNextRouteElement();

        // The route MUST, at this time, be an alternating sequence of Places and PathSegments, so enforce the new
        // element to be a place
        assert(active.isPlace);

        // Check that the new route place matches the place in the map
        if (active.place.getId() != state.placeId) {
            std::cerr << "ERROR:GoalMonitor: Moved to a place not on the planned route! Expected:"
                      << active.place.getId() << " Actual:" << state.placeId << '\n';

            // TODO: So the robot screwed up, now what?
        }
    }

    // If still on a path, then nothing to consider for now
}


void GoalMonitor::updatePlaceProgress(const hssh::GlobalLocation& state)
{
    if (state.onPath) {
        activateNextRouteElement();

        // The route MUST, at this time, be an alternating sequence of Places and PathSegments, so enforce the new
        // element to be a path segment
        assert(active.isPathSegment);

        // Confirm that the two ends of the path segment match the path ends in the robot state. If not, the robot isn't
        // on the correct path segment
        if (((active.segment.getPlusTransition().placeId != state.pathState.entryPlaceId)
             && (active.segment.getMinusTransition().placeId != state.pathState.targetPlaceId))
            && ((active.segment.getMinusTransition().placeId != state.pathState.entryPlaceId)
                && (active.segment.getPlusTransition().placeId != state.pathState.targetPlaceId))) {
            std::cerr << "ERROR:GoalMonitor: Moved onto a path segment with an incorrect place at the other end!\n"
                      << "Expected (P->M):" << active.segment.getPlusTransition().placeId << "->"
                      << active.segment.getMinusTransition().placeId << " Actual:" << state.pathState.targetPlaceId
                      << "->" << state.pathState.entryPlaceId << '\n';

            // TODO: How to correct going onto the wrong path?
        }
    }

    // If still at a place, then all is fine
}


void GoalMonitor::activateNextRouteElement(void)
{
    visited.push_back(active);

    if (!remaining.empty()) {
        active = remaining.front();
        remaining.erase(remaining.begin());
    }
}

}   // namespace planner
}   // namespace vulcan
