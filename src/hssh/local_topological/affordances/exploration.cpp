/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration.cpp
* \author   Collin Johnson
*
* Definition of ExplorationAffordance.
*/

#include "hssh/local_topological/affordances/exploration.h"
#include "hssh/local_topological/affordance_visitor.h"

namespace vulcan
{
namespace hssh
{
    
ExplorationAffordance::ExplorationAffordance(void)
{
    
}


ExplorationAffordance::ExplorationAffordance(const Frontier& frontier)
: NavigationAffordance(pose_t(frontier.exitPoint.x, frontier.exitPoint.y, frontier.direction))
, frontier_(frontier)
{
}


void ExplorationAffordance::accept(NavigationAffordanceVisitor& visitor) const
{
    visitor.visitExploration(*this);
}

}
}
