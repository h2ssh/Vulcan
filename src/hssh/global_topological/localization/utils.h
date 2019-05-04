/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     utils.h
* \author   Collin Johnson
* 
* Declaration of utility functions for localization:
* 
*   - next_path_event : search for the next path event that occurred given the current event count
*   - apply_turn_around_events : finds the topological direction after all path events are applied
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_UTILS_H
#define HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_UTILS_H

#include <hssh/global_topological/utils/visit.h>

namespace vulcan
{
namespace hssh 
{

class TopologicalState;

/**
* next_path_event finds the iterator associated with the next path event to occur for a visit given the number of events
* that have been considered so far.
* 
* \param    state           State to which the visit applies
* \param    visit           Visit with the path events
* \return   An iterator to the next path event that occurred during the visit. If all events have been considered
*   (including the case where no such events have occurred), the returned iter == visit.endPathEvents.
*/
TopologicalVisit::PathEventIter next_path_event(const TopologicalState& state,
                                                const TopologicalVisit& visit);

/**
* apply_turn_around_events applies a series of turn around events to an initial direction to determine the final
* direction of the robot after the last event has occurred.
* 
* \param    initial         Initial direction of the robot along the path 
* \param    beginEvent      Start of the turn around events to apply 
* \param    endEvent        End of the turn around events to apply
* \return   Direction of the robot after all the turn around events have been considered.
*/
TopoDirection apply_turn_around_events(TopoDirection initial,
                                       TopologicalVisit::PathEventIter beginEvent,
                                       TopologicalVisit::PathEventIter endEvent);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_UTILS_H
