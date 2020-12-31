/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     drive_wheel.h
 * \author   Jong Jin Park
 *
 * Declaration of drive_wheel_t, which contains state variables for an individual drive wheel connected to a motor
 */

#ifndef ROBOT_DRIVE_WHEEL_H
#define ROBOT_DRIVE_WHEEL_H

#include <cstdint>
#include <iosfwd>

namespace vulcan
{

/**
 * drive_wheel_t represents the current speed of and motor acceleration acting on a drive wheel.
 */
struct drive_wheel_t
{
    int64_t timestamp;   ///< Time at which the speed was measured from encoder
    float speed;         ///< estimated linear speed of drive wheels
    float motorAccel;    ///< estimated acceleration of the linear speed resulting from the motor torque

    // NOTE: motorAccel is not total acceleration. The total acceleration is a sum of acceleration from the motor,
    // friction and other sources.

    explicit drive_wheel_t(float speed = 0, float motorAccel = 0) : timestamp(0), speed(speed), motorAccel(motorAccel)
    {
    }

    explicit drive_wheel_t(int64_t timestamp, float speed = 0, float motorAccel = 0)
    : timestamp(timestamp)
    , speed(speed)
    , motorAccel(motorAccel)
    {
    }
};

/**
 * differential_drive_wheels_t holds the drive wheels information for a differentially-driven robot.
 */
struct differential_drive_wheels_t
{
    drive_wheel_t leftWheel;
    drive_wheel_t rightWheel;
};


// Output operator: (wheelSpeed, motorAccel)
std::ostream& operator<<(std::ostream& out, const drive_wheel_t& driveWheel);


// Serialization support
template <class Archive>
void serialize(Archive& ar, drive_wheel_t& wheel)
{
    ar(wheel.timestamp, wheel.speed, wheel.motorAccel);
}

template <class Archive>
void serialize(Archive& ar, differential_drive_wheels_t& wheels)
{
    ar(wheels.leftWheel, wheels.rightWheel);
}

}   // namespace vulcan

#endif   // ROBOT_DRIVE_WHEEL_H
