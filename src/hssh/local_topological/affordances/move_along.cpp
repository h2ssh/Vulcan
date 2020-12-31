/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     move_along.cpp
* \author   Collin Johnson
*
* Definition of MoveAlongAffordance.
*/

#include "hssh/local_topological/affordances/move_along.h"
#include "hssh/local_topological/affordance_visitor.h"

namespace vulcan
{
namespace hssh
{

pose_t create_target_based_on_direction(const Point<float>& plusEndpoint,
                                               const Point<float>& minusEndpoint,
                                               TopoDirection             direction);


MoveAlongAffordance::MoveAlongAffordance(void)
{
}


MoveAlongAffordance::MoveAlongAffordance(const Point<float>& plusEndpoint,
                                         const Point<float>& minusEndpoint,
                                         TopoDirection             direction)
: NavigationAffordance(create_target_based_on_direction(plusEndpoint, minusEndpoint, direction))
, direction_(direction)
{
}


void MoveAlongAffordance::accept(NavigationAffordanceVisitor& visitor) const
{
    visitor.visitMoveAlong(*this);
}


pose_t create_target_based_on_direction(const Point<float>& plusEndpoint,
                                               const Point<float>& minusEndpoint,
                                               TopoDirection             direction)
{
    /*
    * Moving in the plus direction means moving towards the plus endpoint, hence the plus endpoint is the target and
    * the heading is from minus to plus. The opposite is the case for moving in the minus direction.
    */
    if(direction == TopoDirection::plus)
    {
        return pose_t{plusEndpoint, angle_to_point(minusEndpoint, plusEndpoint)};
    }
    else // if(direction == TopoDirection::minus)
    {
        return pose_t{minusEndpoint, angle_to_point(plusEndpoint, minusEndpoint)};
    }
}

}
}
