/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file    localizer.h
* \author  Collin Johnson
* 
* Declaration of LocalTopoLocalizer.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCALIZER_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCALIZER_H

#include "hssh/local_topological/event.h"
#include "hssh/local_topological/event_visitor.h"
#include "hssh/local_topological/location.h"

namespace vulcan 
{
namespace hssh 
{
    
class LocalTopoMap;

/**
* LocalTopoLocalizer determines the current location of the robot within the local topo map. The location is found by
* checking the outgoing events for AreaTransitionsEvents. Whenever a transition occurs, the robot's location changes.
* 
* Currently, the location only changes with the area, but further information about motion within a single area could
* prove useful.
*/
class LocalTopoLocalizer : public LocalAreaEventVisitor
{
public:
    
    /**
    * localizeRobot uses the current events to determine if there's a new location for the robot in the local topology.
    * 
    * \param    events          All events that occurred on the current update
    * \param    map             Map of the local topology
    * \return   The robot's current location in the map. If there are no events, then the location doesn't change.
    */
    LocalLocation localizeRobot(const LocalAreaEventVec& events, const LocalTopoMap& map);
    
    // LocalAreaEventVisitor interface
    void visitAreaTransition(const AreaTransitionEvent& event) override;
    void visitTurnAround(const TurnAroundEvent& event) override;

private:

    LocalLocation location_;
    int mapId_;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCALIZER_H
