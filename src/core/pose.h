/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.h
* \author   Collin Johnson
*
* Declaration of pose_t, pose_distribution_t, and associated helper functions. The pose type
* used by all of the Vulcan code.
*/

#ifndef CORE_POSE_H
#define CORE_POSE_H

#include <core/position.h>
#include <core/angle_functions.h>
#include <system/message_traits.h>
#include <cstdint>
#include <iosfwd>

namespace vulcan
{

struct pose_distribution_t;

/**
* pose_t represents the robot pose in Cartesian coordinates in the current global
* frame of reference.
*/
struct pose_t
{
    int64_t timestamp;

    float x;
    float y;
    float theta;

    pose_t(void)
    : timestamp(0)
    , x(0)
    , y(0)
    , theta(0)
    {
    }

    pose_t(float x, float y, float theta)
    : timestamp(0)
    , x(x)
    , y(y)
    , theta(theta)
    {
    }

    explicit pose_t(position_t position, float theta = 0.0f)
    : timestamp(0)
    , x(position.x)
    , y(position.y)
    , theta(theta)
    {
    }

    pose_t(int64_t timestamp, float x, float y, float theta)
    : timestamp(timestamp)
    , x(x)
    , y(y)
    , theta(theta)
    {
    }

    pose_t(const pose_distribution_t& distribution);

    // Some helper functions for converting reference frames, etc.
    /**
    * toPoint converts the pose to a point. Simply chop off the theta.
    */
    position_t toPoint(void) const { return position_t(x, y); }

    /**
    * flip rotates the pose by 180 degrees.
    */
    pose_t flip(void) const { return pose_t(x, y, angle_sum(theta, M_PI)); }

    /**
    * transformToNewFrame applies a transform to the pose and returns the resulting pose.
    *
    * \param    newFrame    New reference frame for the pose (newFrame is the offset from (0,0,0) in the current frame)
    * \return   Pose transformed into the new reference frame.
    */
    pose_t transformToNewFrame(const pose_t& newFrame) const;

    /**
    * compound applies the compound operator to transform this pose to a new location relative to the provided origin.
    *
    * \param    origin          Origin pose of the compound, i.e. on the right of the circle plus
    * \return   this circle-plus origin.
    */
    pose_t compound(const pose_t& origin) const;
};

/**
* pose_6dof_t is a full 6-DOF pose. While most of the robot functionality only needs the simpler 3-DOF
* pose, some sensors, like the laser rangefinders, exist at different heights and rotations, so the full
* description of their pose is needed when converting values into the robot's frame of reference.
*/
struct pose_6dof_t
{
    int64_t timestamp;

    float x;
    float y;
    float z;

    float phi;      ///< Pitch
    float rho;      ///< Roll
    float theta;    ///< Yaw

    pose_6dof_t(void)
        : timestamp(0)
        , x(0)
        , y(0)
        , z(0)
        , phi(0)
        , rho(0)
        , theta(0)
    {
    }

    pose_6dof_t(float x, float y, float theta)
        : timestamp(0)
        , x(x)
        , y(y)
        , z(0)
        , phi(0)
        , rho(0)
        , theta(theta)
    {
    }

    pose_6dof_t(int64_t timestamp, float x, float y, float z, float phi, float rho, float theta)
        : timestamp(timestamp)
        , x(x)
        , y(y)
        , z(z)
        , phi(phi)
        , rho(rho)
        , theta(theta)
    {
    }

    // Allow implicit conversion from a pose
    pose_6dof_t(const pose_t& pose)
    : pose_6dof_t(pose.timestamp, pose.x, pose.y, 0.0f, 0.0f, 0.0f, pose.theta)
    {
    }

    /**
    * to2DPosition retrieves the 2D position, (x,y), of the 6dof pose.
    */
    position_t to2DPosition(void) const { return position_t(x, y); }

    /**
    * toPose retrieves the 3-dof pose (x, y, theta) of the full 6dof pose.
    */
    pose_t toPose(void) const { return pose_t(timestamp, x, y, theta); }
};

// Various operator overloads
bool          operator==(const pose_t& lhs, const pose_t& rhs);
bool          operator!=(const pose_t& lhs, const pose_t& rhs);
std::ostream& operator<<(std::ostream& out, const pose_t& pose);
std::ostream& operator<<(std::ostream& out, const pose_6dof_t& pose);

/**
* transform_pose_to_new_frame transforms a pose to a new reference frame.
*
* \param    pose        Pose to be transformed
* \param    newFrame    New reference frame for the pose (newFrame is the offset from (0,0,0) in the current frame)
* \return   Pose transformed into the new reference frame.
*/
pose_t transform_pose_to_new_frame(const pose_t& pose, const pose_t& newFrame);

/**
* interpolate_pose calculates an estimate of the robot pose given two robot poses
* and a time in the time interval between the poses.
*
* \pre  priorPose.timestamp < desiredPoseTime < currentPose.timestamp
*/
pose_t interpolate_pose(const pose_t& priorPose, const pose_t& currentPose, int64_t desiredPoseTime);

// Serialization support -- both types are just PODs
template <class Archive>
void serialize(Archive& ar, pose_t& pose)
{
    ar (pose.timestamp,
        pose.x,
        pose.y,
        pose.theta);
}

template <class Archive>
void serialize(Archive& ar, pose_6dof_t& pose, const unsigned int version)
{
    ar (pose.timestamp,
        pose.x,
        pose.y,
        pose.z,
        pose.theta,
        pose.rho,
        pose.phi);
}

} // namespace vulcan

DEFINE_SERIALIZATION_MSG_TRAITS(pose_t, ("ROBOT_POSE"), old_message_tag)

#endif // CORE_POSE_H
