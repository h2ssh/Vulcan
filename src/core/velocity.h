/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     velocity.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of velocity_t and acceleration_t, with linear and angular components.
*/

#ifndef CORE_VELOCITY_H
#define CORE_VELOCITY_H

#include <iosfwd>
#include <cstdint>

namespace vulcan
{

/**
* velocity_t represents the current velocity of the robot with linear and angular components.
* Only magnitude of the velocity vectors are specified, since direction is always aligned with the current robot pose.
*/
struct velocity_t
{
    int64_t timestamp;      ///< Time at which this velocity was measured
    float   linear;
    float   angular;

    explicit velocity_t(float linear = 0, float angular = 0, int64_t timestamp = 0)
    : timestamp(timestamp)
    , linear(linear)
    , angular(angular)
    {
    }
};


/**
* acceleration_t represents the current acceleration of the robot with linear and angular components.
* Only magnitude of the velocity vectors are specified, since direction is always aligned with the current robot pose.
*/
struct acceleration_t
{
    int64_t timestamp;      ///< Time at which this velocity was measured
    float   linear;
    float   angular;

    explicit acceleration_t(float linear = 0, float angular = 0, int64_t timestamp = 0)
    : timestamp(timestamp)
    , linear(linear)
    , angular(angular)
    {
    }
};

// Output operator: (linear, angular)
std::ostream& operator<<(std::ostream& out, const velocity_t& v);
std::ostream& operator<<(std::ostream& out, const acceleration_t& a);

// Serialization support
template <class Archive>
void serialize(Archive& ar, velocity_t& vel)
{
    ar (vel.timestamp,
        vel.linear,
        vel.angular);
}

template <class Archive>
void serialize(Archive& ar, acceleration_t& accel)
{
    ar (accel.timestamp,
        accel.linear,
        accel.angular);
}

} // namespace vulcan

#endif // CORE_VELOCITY_H
