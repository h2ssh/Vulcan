/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file    localizer.cpp
* \author  Collin Johnson
* 
* Definition of LocalTopoLocalizer.
*/

#include <hssh/local_topological/localizer.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/events/area_transition.h>

namespace vulcan
{
namespace hssh
{

LocalLocation LocalTopoLocalizer::localizeRobot(const LocalAreaEventVec& events, const LocalTopoMap& map)
{
    mapId_ = map.mapId();
    
    for(auto& e : events)
    {
        e->accept(*this);
    }
    
    return location_;
}

// LocalAreaEventVisitor interface
void LocalTopoLocalizer::visitAreaTransition(const AreaTransitionEvent& event)
{
    if(event.transitionGateway())
    {
        location_ = LocalLocation(mapId_, event.enteredArea()->id(), event.transitionGateway().get());
    }
    else
    {
        location_ = LocalLocation(mapId_, event.enteredArea()->id());
    }
}


void LocalTopoLocalizer::visitTurnAround(const TurnAroundEvent& event)
{
    // Ignore turn around event as it doesn't change the location
}

}
}