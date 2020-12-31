/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_to_global.h
* \author   Collin Johnson
*
* Declaration of utility functions that convert from the local topological representation of an area to a global
* topological representation of the area:
*
*   - find_place_exit_transition finds the exit transition in the provided cycle that corresponds to the exit transition in
*       the corresponding topological visit.
*   - find_relative_place_exit_index: find index offset of the exit transition relative to the entry transition in the
*       standard counter-clockwise order around the cycle
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_UTILS_LOCAL_TO_GLOBAL_H
#define HSSH_GLOBAL_TOPOLOGICAL_UTILS_LOCAL_TO_GLOBAL_H

#include "hssh/global_topological/transition_cycle.h"
#include <boost/optional.hpp>

namespace vulcan
{
namespace hssh
{

class GlobalPathSegment;
class GlobalPlace;
class GlobalTransition;
class LocalArea;
class TopologicalVisit;

/**
* create_global_place_from_local_place creates a GlobalPlace representation of the corresponding LocalPlace.
*
* \param    globalArea          GlobalArea with the identifying information for the area in the topological map
* \param    entryGateway        Entry gateway to the local area
* \param    localArea           LocalArea describing the area in the LPM
* \return   GlobalPlace representation of the local area along with the entry transition.
*/
std::pair<GlobalPlace, GlobalTransition> create_global_place_from_local_place(const GlobalArea& globalArea,
                                                                              boost::optional<Gateway> entryGateway,
                                                                              const LocalArea& localArea);

/**
* create_global_path_segment_from_local_path_segemnt creates a GlobalPathSegment representation of the corresponding
* LocalPathSegment.
*
* \param    globalArea          GlobalArea with the identifying information for the area in the topological map
* \param    entryGateway        Entry gateway to the local area (if one exists)
* \param    localArea           LocalArea describing the area in the LPM
* \param    isExplored          Flag indicating if this path segment was fully explored by the visit that created it
* \return   GlobalPathSegment representation of the local area along with the entry transition.
*/
std::pair<GlobalPathSegment, GlobalTransition> create_global_path_segment_from_local_path_segment(
    const GlobalArea& globalArea,
    boost::optional<Gateway> entryGateway,
    const LocalArea& localArea,
    bool isExplored
);

/**
* find_place_exit_transition finds the exit transition in the provided cycle that corresponds to the exit transition in the
* corresponding topological visit. The function works by matching the entry transition with a transition in the visit
* and then iterating around the cycle the same number of steps as the exit transition stored in visit to determine
* which transition was exited.
*
* \param    cycle           Cycle being visited by robot
* \param    entry           Entry transition to the place
* \param    visit           Completed visit made by the robot, i.e. has an exit
* \return   The transition through which the robot exited. If the cycles aren't comparable, then an invalid transition
*   is returned.
*/
GlobalTransition find_place_exit_transition(const GlobalTransitionCycle& cycle,
                                            const GlobalTransition& entry,
                                            const TopologicalVisit& visit);

/**
* find_relative_place_exit_index finds the index of the exit transition relative to the entry transition. The relative
* index is equivalent to the number of times next() would need to be called starting at the entry transition to reach
* the exit transition.
*
* \param    visit           Visit in which to find the exit index
* \return   The relative index of the exit area from the entry area in the standard counter-clockwise order around the
*   transition cycle.
*/
int find_relative_place_exit_index(const TopologicalVisit& visit);

/**
* find_path_exit_transition finds the exit transition for a path. The returned transition is either the transition in
* the direction of motion for the exit or a transition along the sequence. If a transition along the sequence, then
* the returned transition may be a frontier transition, which the robot hasn't entered before, meaning that the robot
* has gone through a door that was previously closed.
*
* \param    pathSegment             Path segment the robot was in
* \param    entry                   Entry transition for the segment
* \param    visit                   Visit with the entry and exit for the path segment
* \return   Transition through which the robot exited the area.
*/
GlobalTransition find_path_exit_transition(const GlobalPathSegment& pathSegment,
                                           const GlobalTransition& entry,
                                           const TopologicalVisit& visit);

/**
* path_endpoints_were_swapped checks if the path endpoints were swapped between the entry of a path segment and its
* exit. They are swapped if the plus is the same as the minus and vice versa between the exit version and the version
* being stored in the TopologicalMap.
*
* This method must be used because the underlying gateway information is the only way to determine for sure that a
* transition matches and was swapped.
*
* \param    entryPathSegment        Path segment as seen on entry
* \param    entryTransition         Transition seen on entry
* \param    exitPathSegment         Path segment at the exit
* \param    exitTransition          Transition for entry seen at the exit
* \param    visit                   Visit encompassing the full journey to the path segment
* \return   True if entryPathSegment.plus == exitPathSegment.minus.
*/
bool path_endpoints_were_swapped(const GlobalPathSegment& entryPathSegment,
                                 const GlobalTransition& entryTransition,
                                 const GlobalPathSegment& exitPathSegment,
                                 const GlobalTransition& exitTransition,
                                 const TopologicalVisit& visit);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_UTILS_LOCAL_TO_GLOBAL_H
