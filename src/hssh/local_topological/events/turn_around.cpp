/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     turn_around.cpp
* \author   Collin Johnson
* 
* Definition of TurnAroundEvent.
*/

#include "hssh/local_topological/events/turn_around.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/event_visitor.h"
#include <sstream>

namespace vulcan
{
namespace hssh
{
    
TurnAroundEvent::TurnAroundEvent(int64_t                           timestamp,
                                 std::shared_ptr<LocalPathSegment> path, 
                                 TopoDirection                     current, 
                                 TopoDirection                     previous,
                                 const LocalPose&                  turnAroundPose)
: LocalAreaEvent(timestamp, turnAroundPose, true)
, pathSegment_(path)
, current_(current)
, previous_(previous)
{
    areaIds_.push_back(path->id());
}


std::string TurnAroundEvent::description(void) const 
{ 
    std::ostringstream str;
    str << "TurnAround " << sequenceId() << ": " << pathSegment_->description() 
        << " Current:" << current_ << " Previous: " << previous_;
    return str.str();
}


void TurnAroundEvent::accept(LocalAreaEventVisitor& visitor) const
{ 
    visitor.visitTurnAround(*this); 
}


LocalAreaEventPtr TurnAroundEvent::clone(void) const
{
    return LocalAreaEventPtr(new TurnAroundEvent(*this));
}

}
}
