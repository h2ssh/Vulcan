/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_state.h
* \author   Collin Johnson
*
* Definition of object_state_t.
*/

#ifndef TRACKER_OBJECT_STATE_H
#define TRACKER_OBJECT_STATE_H

#include <tracker/object_boundary.h>

namespace vulcan
{
namespace tracker
{

/**
* object_motion_state_t defines estimated motion properties of the object: (x, y, xVel, yVel, xAccel, yAccel)
*/
struct object_motion_state_t
{
    float x = 0.0f;
    float y = 0.0f;
    float xVel = 0.0f;
    float yVel = 0.0f;
    float xAccel = 0.0f;
    float yAccel = 0.0f;

    object_motion_state_t(void) = default;
    object_motion_state_t(float xPos, float yPos, float xVel, float yVel, float xAccel = 0.0f, float yAccel = 0.0f)
    : x(xPos)
    , y(yPos)
    , xVel(xVel)
    , yVel(yVel)
    , xAccel(xAccel)
    , yAccel(yAccel)
    {
    }
};

/**
* object_state_t contains the state of an object -- (time, motion, boundary).
*/
struct object_state_t
{
    int64_t timestamp;
    object_motion_state_t motion;
    ObjectBoundary boundary;
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, object_motion_state_t& motion)
{
    ar( motion.x,
        motion.y,
        motion.xVel,
        motion.yVel,
        motion.xAccel,
        motion.yAccel);
}

template <class Archive>
void serialize(Archive& ar, object_state_t& state)
{
    ar( state.timestamp,
        state.motion,
        state.boundary);
}

} // namespace vulcan
} // namespace tracker

#endif // TRACKER_OBJECT_STATE_H
