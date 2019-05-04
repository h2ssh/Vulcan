/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     steady.cpp
* \author   Collin Johnson
*
* Definition of SteadyMotion.
*/

#include <tracker/motions/steady.h>
#include <tracker/motions/visitor.h>
#include <tracker/laser_object.h>
#include <tracker/object_state.h>
#include <iostream>
#include <cassert>

#define DEBUG_STATE

namespace vulcan
{
namespace tracker
{

SteadyMotion::SteadyMotion(void)
{
}


SteadyMotion::SteadyMotion(int64_t timestamp, Position initialPosition, const tracking_filter_params_t& filterParams)
: ObjectMotion(initialPosition, velocity_t(0.0f, 0.0f))
, motionTracker_(timestamp, initialPosition, filterParams)
{
}


SteadyMotion::~SteadyMotion(void)
{
    // For std::unique_ptr
}


ObjectMotionStatus SteadyMotion::modelStatus(void) const
{
    return ObjectMotionStatus::valid;
}


void SteadyMotion::accept(ObjectMotionVisitor& visitor) const
{
    visitor.visitSteady(*this);
}


std::unique_ptr<ObjectMotion> SteadyMotion::clone(void) const
{
    return std::make_unique<SteadyMotion>(*this);
}


object_motion_state_t SteadyMotion::updateMotionEstimate(const LaserObject& object)
{
    motionTracker_.update(object);

    auto slowState = motionTracker_.slowState();
    auto fastState = motionTracker_.fastState();

    // The default state is a combination of the fast and slow states.
    // The fast position should be used because it is what is used for collision detection.
    // The slow velocity is more stable, so it should be used for prediction.
    // The fast acceleration should be used because acceleration is a very transient value
    object_motion_state_t combinedState = fastState;
    combinedState.xVel = slowState.xVel;
    combinedState.yVel = slowState.yVel;

    return combinedState;
}


Position SteadyMotion::estimateFuturePosition(int deltaTimeMs) const
{
    double deltaTime = deltaTimeMs / 1000.0;

    Position estimatedPos = Position(motionTracker_.fastState().x, motionTracker_.fastState().y);
    estimatedPos.x += motionTracker_.slowState().x * deltaTime;
    estimatedPos.y += motionTracker_.slowState().y * deltaTime;

    return estimatedPos;
}

} // namespace tracker
} // namespace vulcan
