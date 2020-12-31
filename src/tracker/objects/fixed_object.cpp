/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     fixed_object.cpp
 * \author   Collin Johnson
 *
 * Definition of FixedObject.
 */

#include "tracker/objects/fixed_object.h"
#include "tracker/laser_object.h"

namespace vulcan
{
namespace tracker
{

void FixedObject::updateModel(const LaserObject& object)
{
    model_.updateModel(object.minErrorBoundary());
    state_.timestamp = object.timestamp();
    state_.motion.x = model_.movingPosition().x;
    state_.motion.y = model_.movingPosition().y;
    state_.boundary = model_.boundary();

    // Calculate the velocity based on the updated trajectory
    auto velocity = estimateVelocity(object.timestamp());
    state_.motion.xVel = velocity.x;
    state_.motion.yVel = velocity.y;
}


void FixedObject::setGoals(const ObjectGoalDistribution& goals)
{
    goals_ = goals;
}


float FixedObject::overlapWithObject(const LaserObject& object) const
{
    return std::get<0>(model_.boundary().distanceFromBoundary(object.begin(), object.end()));
}


Position FixedObject::position(void) const
{
    return model_.movingPosition();
}


velocity_t FixedObject::velocity(void) const
{
    return velocity_t(state_.motion.xVel, state_.motion.yVel);
}


object_motion_state_t FixedObject::motionState(void) const
{
    return state_.motion;
}


ObjectBoundary FixedObject::boundary(void) const
{
    return model_.boundary();
}


double FixedObject::radius(void) const
{
    return boundary().circleApproximation().radius();
}


FixedObject::FixedObject(int64_t timestamp, const FixedObjectModel& model)
: lastUpdateTime_(timestamp)
, totalTimeSeen_(0)
, model_(model)
{
}


}   // namespace tracker
}   // namespace vulcan
