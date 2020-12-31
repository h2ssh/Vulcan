/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     transition.cpp
* \author   Collin Johnson
* 
* Definition of TransitionAffordance.
*/

#include "hssh/local_topological/affordances/transition.h"
#include "hssh/local_topological/affordance_visitor.h"

namespace vulcan
{
namespace hssh
{
    
TransitionAffordance::TransitionAffordance(void)
{
}


TransitionAffordance::TransitionAffordance(const Gateway& gateway, AreaType type)
: NavigationAffordance(pose_t(gateway.center().x, gateway.center().y, gateway.direction()))
, gateway_(gateway)
, type_(type)
{
}


void TransitionAffordance::accept(NavigationAffordanceVisitor& visitor) const
{
    visitor.visitTransition(*this);
}

}
}
