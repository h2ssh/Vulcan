/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_transition.cpp
* \author   Collin Johnson
* 
* Definition of AreaTransitionEvent.
*/

#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/event_visitor.h"
#include <sstream>
#include <cassert>

namespace vulcan
{
namespace hssh
{
    
AreaTransitionEvent::AreaTransitionEvent(int64_t                    timestamp, 
                                         const LocalPose&           pose,
                                         std::shared_ptr<LocalArea> exited, 
                                         std::shared_ptr<LocalArea> entered, 
                                         const Gateway&             gateway)
: LocalAreaEvent(timestamp, pose, true)
, entered_(entered)
, exited_ (exited)
, gateway_(gateway)
{
    assert(exited_);
    assert(entered_);

    areaIds_.push_back(exited->id());
    areaIds_.push_back(entered->id());
}


AreaTransitionEvent::AreaTransitionEvent(int64_t                    timestamp,
                                         const LocalPose&           pose,
                                         std::shared_ptr<LocalArea> entered)
: LocalAreaEvent(timestamp, pose, true)
, entered_(entered)
, gateway_(boost::none)
{
    areaIds_.push_back(entered->id());
}


Id AreaTransitionEvent::exitedId(void) const
{
    return exited_ ? exited_->id() : kInvalidId;
}


Id AreaTransitionEvent::enteredId(void) const
{
    assert(entered_);
    return entered_->id();
}


void AreaTransitionEvent::accept(LocalAreaEventVisitor& visitor) const
{ 
    visitor.visitAreaTransition(*this);
}


std::string AreaTransitionEvent::description(void) const
{ 
    std::ostringstream str;
    str << "AreaTransition " << sequenceId() 
        << ": Exited " << (exited_? exited_->description() : "-1")
        << " Entered:" << entered_->description();
    return str.str();
}


LocalAreaEventPtr AreaTransitionEvent::clone(void) const
{
    return LocalAreaEventPtr(new AreaTransitionEvent(*this));
}

} // namespace hssh
} // namespace vulcan
