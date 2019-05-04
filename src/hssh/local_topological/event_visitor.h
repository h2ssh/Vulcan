/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     event_visitor.h
* \author   Collin Johnson
* 
* Declaration of LocalEventVisitor interface.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENT_VISITOR_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENT_VISITOR_H

namespace vulcan
{
namespace hssh
{

class AreaTransitionEvent;
class TurnAroundEvent;

/**
* LocalAreaEventVisitor is the Visitor interface for the LocalAreaEvent class hierarchy. There is
* a virtual visitXXXX method for each event.
*/
class LocalAreaEventVisitor
{
public:
    
    virtual ~LocalAreaEventVisitor(void) { }
    
    virtual void visitAreaTransition(const AreaTransitionEvent& event) = 0;
    virtual void visitTurnAround    (const TurnAroundEvent&     event) = 0;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENT_VISITOR_H
