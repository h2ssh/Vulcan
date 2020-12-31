/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     circle.cpp
 * \author   Collin Johnson
 *
 * Definition of CircleBoundary.
 */

#include "tracker/boundaries/circle.h"

namespace vulcan
{
namespace tracker
{

CircleBoundary::CircleBoundary(void)
: radius_(5.0)        // make the initial radius ridiculous, so it is obvious if a default-constructed boundary is used
, totalWeight_(0.0)   // no initial weight means the first measurement will create the initial radius
{
}


void CircleBoundary::addEstimate(Arc arc)
{
    double arcWeight = arc.range().extent / M_PI / 2.0;
    double newTotalWeight = totalWeight_ + arcWeight;

    if (newTotalWeight > 0.0) {
        radius_ = ((radius_ * totalWeight_) + (arc.radius() * arcWeight)) / newTotalWeight;
        totalWeight_ = newTotalWeight;
    }
}

}   // namespace tracker
}   // namespace vulcan
