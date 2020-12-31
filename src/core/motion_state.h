/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_state.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of motion_state_t, a composition of the robot pose, pose uncertainty,
* robot velocity, and the states of the two motor-driven drive wheels.
*/

#ifndef CORE_MOTION_STATE_H
#define CORE_MOTION_STATE_H

#include "core/pose.h"
#include "core/pose_distribution.h"
#include "core/velocity.h"
#include "core/drive_wheel.h"
#include "system/message_traits.h"
#include <cassert>

namespace vulcan
{

/**
* motion_state_t defines the full motion state of a differentially driven vehicle.
* The motion state is composed of robot pose, uncertainty, velocity and the states
* of the two motor-driven drive wheels.
*/
struct motion_state_t
{
    int64_t timestamp;

    pose_t                      pose;
    pose_distribution_t         poseDistribution;
    velocity_t                  velocity;
    acceleration_t              acceleration;
    differential_drive_wheels_t differentialWheels;

    bool haveDistribution;      ///< Flag indicating if this instance of motion_state_t has a valid poseDistribution
    bool haveWheels;            ///< Flag indicating if this instance of motion_state_t has a valid differentialWheels parameter

    motion_state_t(void)
    : timestamp(0)
    {
        haveDistribution = haveWheels = false;
    }

    motion_state_t(const pose_t&     pose,
                   const velocity_t& velocity)
    : timestamp(pose.timestamp)
    , pose(pose)
    , velocity(velocity)
    {
        haveDistribution = haveWheels = false;
    }

    motion_state_t(const pose_distribution_t& poseDistribution,
                   const velocity_t&          velocity)
    : timestamp(poseDistribution.timestamp)
    , pose(poseDistribution.toPose())
    , poseDistribution(poseDistribution)
    , velocity(velocity)
    {
        haveDistribution = true;
        haveWheels       = false;
    }

    motion_state_t(const pose_t&                      pose,
                   const velocity_t&                  velocity,
                   const differential_drive_wheels_t& wheels)
    : timestamp(pose.timestamp)
    , pose(pose)
    , velocity(velocity)
    , differentialWheels(wheels)
    {
        haveDistribution = false;
        haveWheels       = true;
    }

    motion_state_t(const pose_distribution_t&         poseDistribution,
                   const velocity_t&                  velocity,
                   const differential_drive_wheels_t& wheels)
    : timestamp(poseDistribution.timestamp)
    , pose(poseDistribution.toPose())
    , poseDistribution(poseDistribution)
    , velocity(velocity)
    , differentialWheels(wheels)
    {
        haveDistribution = haveWheels = true;
    }

    motion_state_t(const pose_t&                      pose,
                   const velocity_t&                  velocity,
                   const acceleration_t&              acceleration,
                   const differential_drive_wheels_t& wheels)
    : timestamp(pose.timestamp)
    , pose(pose)
    , velocity(velocity)
    , acceleration(acceleration)
    , differentialWheels(wheels)
    {
        haveDistribution = false;
        haveWheels       = true;
    }

    motion_state_t(const pose_distribution_t&         poseDistribution,
                   const velocity_t&                  velocity,
                   const acceleration_t&              acceleration,
                   const differential_drive_wheels_t& wheels)
    : timestamp(poseDistribution.timestamp)
    , pose(poseDistribution.toPose())
    , poseDistribution(poseDistribution)
    , velocity(velocity)
    , acceleration(acceleration)
    , differentialWheels(wheels)
    {
        haveDistribution = haveWheels = true;
    }
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, motion_state_t& state)
{
    ar (state.timestamp,
        state.pose,
        state.poseDistribution,
        state.velocity,
        state.acceleration,
        state.differentialWheels,
        state.haveDistribution,
        state.haveWheels);
}

} // namespace vulcan

DEFINE_SYSTEM_MESSAGE(motion_state_t, ("ROBOT_MOTION_STATE"))

#endif // CORE_MOTION_STATE_H
