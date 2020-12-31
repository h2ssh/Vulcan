/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stationary.cpp
* \author   Collin Johnson
* 
* Definition of StationaryMotion.
*/

#include "tracker/motions/stationary.h"
#include "tracker/laser_object.h"
#include "tracker/motions/visitor.h"

namespace vulcan 
{
namespace tracker 
{

StationaryMotion::StationaryMotion(void)
{
}


StationaryMotion::StationaryMotion(Position position, float maxStationaryDist)
: ObjectMotion(position, velocity_t(0.0f, 0.0f))
, initialPosition_(position)
, kMaxStationaryDist_(maxStationaryDist)
{
}


void StationaryMotion::accept(ObjectMotionVisitor& visitor) const
{
    visitor.visitStationary(*this);
}


std::unique_ptr<ObjectMotion> StationaryMotion::clone(void) const
{
    return std::make_unique<StationaryMotion>(*this);
}


ObjectMotionStatus StationaryMotion::modelStatus(void) const
{
    // StationaryMotion is valid as long as the object hasn't moved too far from its initially estimated position
    if(distance_between_points(position(), initialPosition_) < kMaxStationaryDist_)
    {
        return ObjectMotionStatus::valid;
    }
    else
    {
        return ObjectMotionStatus::invalid;
    }
}


object_motion_state_t StationaryMotion::updateMotionEstimate(const LaserObject& object)
{
    // A stationary object is never moving and is always located wherever the last detection took place
    return object_motion_state_t(object.center().x, object.center().y, 0.0f, 0.0f);
}


Position StationaryMotion::estimateFuturePosition(int deltaTimeMs) const
{
    // A stationary object never moves, so its future positions are the same as the current position
    return position();
}

}
}
