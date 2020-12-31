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
 * Definition of GlobalTransition.
 */

#include "hssh/global_topological/transition.h"
#include <iomanip>
#include <iostream>

// #define CHECk_TYPE_COMPATIBILITY

namespace vulcan
{
namespace hssh
{

///////////////// GlobalTransition definition ///////////////

GlobalTransition::GlobalTransition(Id id,
                                   GlobalArea plusArea,
                                   GlobalArea minusArea,
                                   NavigationStatus navigable,
                                   ExplorationStatus explored)
: id_(id)
, plusArea_(plusArea)
, minusArea_(minusArea)
, navigable_(navigable)
, explored_(explored)
{
}


GlobalTransition::GlobalTransition(Id id, GlobalArea area)
: GlobalTransition(id, area, GlobalArea(), NavigationStatus::null, ExplorationStatus::explored)
{
}


bool GlobalTransition::valid(void) const
{
    return (id_ != kInvalidId) && ((plusArea_.id() != kInvalidId) || (minusArea_.id() != kInvalidId));
}


GlobalArea GlobalTransition::otherArea(const GlobalArea& thisArea) const
{
    if (thisArea == minusArea_) {
        return plusArea_;
    } else if (thisArea == plusArea_) {
        return minusArea_;
    } else {
        return GlobalArea();
    }
}


GlobalArea GlobalTransition::visitedArea(void) const
{
    return (plusArea().id() == kFrontierId) ? minusArea() : plusArea();
}


///////////////// GlobalTransition operators ///////////////
bool operator==(const GlobalTransition& lhs, const GlobalTransition& rhs)
{
    return lhs.id() == rhs.id();
}


bool operator!=(const GlobalTransition& lhs, const GlobalTransition& rhs)
{
    return !(lhs == rhs);
}

std::ostream& operator<<(std::ostream& out, const GlobalTransition& transition)
{

    out << std::boolalpha << "Transition " << transition.id() << ": Plus:" << transition.plusArea()
        << " Minus:" << transition.minusArea() << " Nav:" << transition.isNavigable()
        << " Exp:" << !transition.isFrontier();
    return out;
}


bool are_similar_transitions(const GlobalTransition& lhs, const GlobalTransition& rhs)
{
#ifdef CHECK_TYPE_COMPATIBILITY
    return (lhs.isNavigable() == rhs.isNavigable())
      && (((lhs.plusArea().type() == rhs.plusArea().type()) && (lhs.minusArea().type() == rhs.minusArea().type()))
          || ((lhs.plusArea().type() == rhs.minusArea().type()) && (lhs.minusArea().type() == rhs.plusArea().type())));
#else
    return lhs.isNavigable() == rhs.isNavigable();
#endif   // CHECK_TYPE_COMPATIBILITY
}

}   // namespace hssh
}   // namespace vulcan
