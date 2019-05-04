/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     localizer.h
* \author   Collin Johnson
*
* Declaration of TopologicalLocalizer.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_TOPOLOGICAL_LOCALIZER_H
#define HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_TOPOLOGICAL_LOCALIZER_H

#include <hssh/global_topological/localization/location_distribution.h>
#include <hssh/global_topological/utils/visit.h>

namespace vulcan
{
namespace hssh
{

struct TopologicalState;

/**
* TopologicalLocalizer is an base class for implementations of topological localization. The localizer takes
* one TopologicalState along with the current visit and then determines a distribution across possible future
* topological states.
* 
* The localizer provides three points of customization:
* 
*   - handleEnteredEvent : decide what to do when the area is entered 
*   - handleExitedEvent : decide what to do when the area is exited 
*   - handleTurnAroundEvents : decide what to do when the robot turns around on a path
* 
* The default behavior for these methods assumes deterministic actions by the robot. The result of actions are:
* 
*   - turning around : the robot deterministically reverses its direction along the path. The new location distribution
*       contains only the current path with the direction reversed.
*   - exiting an area : the robot deterministically exits an area. The exit transition leads to the associated area as
*       determined by the GlobalTransitionCycle or Sequence.
*   - entering an area : the robot has entered the area that was predicted by the previous exited event. The location is
*       the same as what the exited event specified.
*/
class TopologicalLocalizer
{
public:

    virtual ~TopologicalLocalizer(void) { }

    /**
    * localize localizes the robot within the map hypothesis in the tree-of-maps based on some topological visit made
    * by the robot. The new distribution across possible locations is found and returned.
    * 
    * All events that exist within the visit are applied. The general flow should be to localize, which applies the exit,
    * then use the visit to estimate the new possible states. Then apply localize with the next visit.
    *
    * \param    state       State to apply the visit to
    * \param    visit       Topological visit that resulted in the robot moving somewhere new
    * \return   Distribution of all possible global locations for the robot after applying the new visit.
    */
    GlobalLocationDistribution localize(const TopologicalState& state, const TopologicalVisit& visit);
    
protected:
    
    /**
    * verifyEnteredEvent is processed when the robot has entered a new area. The visit will be associated with the entry
    * area, which will have already been known during the exit transition. Thus, the entry and exit events shouldn't
    * produce different states by default. This method verifies that this condition holds.
    * 
    * If other behavior is needed, then it can be provided.
    * 
    * \param    state           State upon entry
    * \param    visit           Visit to apply to the state
    * \return   True if the visit entry state is valid, given the current state.
    */
    virtual bool verifyEnteredEvent(const TopologicalState& state, const TopologicalVisit& visit);
    
    /**
    * handleExitedEvent is processed whenever a visit indicates that the robot has left the particular area. In the case
    * of an area transition, the robot will have entered some new area. The new area is determined based on the
    * transition and the type of area. If in a place, the correct area based on the transition cycle will be set.
    * Otherwise, it will be an end of the path or a destination along the path.
    * 
    * The default behavior is for the area that was entered to be deterministically determined by the topological map.
    * Thus, if the robot is moving along a path, after exiting the path it will be at the area on the other end of the 
    * path with probability 1. If place detection is perfect, then this condition might not hold, in which case this
    * method will need to be overridden.
    * 
    * \param    state           State being exited 
    * \param    location        Estimated location after applying events prior to the exit
    * \param    visit           Visit describing the events of that occurred in the area being exited 
    * \return   A distribution with a single location indicating the next area. If no exited event has occurred yet,
    *   then the distribution is just the current location with max probability.
    */
    virtual GlobalLocationDistribution handleExitedEvent(const TopologicalState& state,
                                                         const GlobalLocation& location,
                                                         const TopologicalVisit& visit);
    
    /**
    * handleTurnAroundEvents is processed whenever the robot has turned around along a path. The turn around can either
    * provide the initial direction of motion along a path, in situations where the robot enters from a transition
    * sequence, or flips the direction.
    * 
    * \param    state           State in which a turn around occurred 
    * \param    beginEvent      First path event to occur 
    * \param    endEvent        Final path event to occur
    * \return   The new direction of the robot after dealing with all of these turn around events.
    */
    virtual TopoDirection handleTurnAroundEvents(const TopologicalState& state,
                                                 TopologicalVisit::PathEventIter beginEvent,
                                                 TopologicalVisit::PathEventIter endEvent);
    
private:
    
    GlobalLocationDistribution handlePathExit(const TopologicalState& state, 
                                              const GlobalLocation& location, 
                                              const TopologicalVisit& visit);
    GlobalLocationDistribution handlePlaceExit(const TopologicalState& state, 
                                               const GlobalLocation& location, 
                                               const TopologicalVisit& visit);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_TOPOLOGICAL_LOCALIZER_H
