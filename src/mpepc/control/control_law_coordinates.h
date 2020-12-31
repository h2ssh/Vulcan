/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     control_law_coordinates.h
 * \author   Jong Jin Park
 *
 * Declaration of approach_direction_t, control_law_coordinates_t, motion_target_t and control_law_output_t.
 */

#ifndef MPEPC_KINEMATIC_CONTROL_LAW_COORDINATES_H
#define MPEPC_KINEMATIC_CONTROL_LAW_COORDINATES_H

#include "core/angle_functions.h"
#include "core/pose.h"
#include <cereal/access.hpp>

namespace vulcan
{

namespace mpepc
{

const float SMALL_RADIUS_M =
  0.05;   // constant determining when the robot is too close to the target to compute the line of sight.
const float DEFAULT_VELOCITY_GAIN = 0.7;   // default value for the velocity gain
const float DEFAULT_K1 = 1.2;              // default value for the first shape parameter
const float DEFAULT_K2 = 3.0;              // default value for the second shape parameter

/**
 * approach_direction_t defines the two directions a target pose can be approached by a non-holonomic
 * vehicle when the radial distance between the vehicle and the target pose is non-zero.
 */
enum approach_direction_t
{
    FORWARD,
    BACKWARD
};

/**
 * control_law_coordinates_t defines the set error coordinates for the current control
 * law used. The error coordinates for the control law, based on the difference between
 * the current robot pose and a target pose, are:
 *
 *     r     : radial distance from current pose to the target. negative when the desired direction of approach is
 * backward. theta : angular difference between the line-of-sight line from robot to waypoint position and the waypoint
 * target orientation delta : angular difference between the line-of-sight line from robot to waypoint position and the
 * robot orientation
 *
 * As the line of sight is indefinite at r = 0, we fix it to the target orientation when the radial distance r is small.
 *
 * When the specified direction of approach is backward, the control law coordinates
 * are computed with the robot and the target pose flipped (rotated by 180 degrees),
 * and the radial coordinate r is given a negative sign, where r is always positive
 * when the direction of approach is forward. This is to allow the optimizer to search
 * in the continuous space of r, with negative r indicating backward approach direction.
 *
 * Thus for the KinematicControlLaw, if r >= 0, the target should be approached with
 * forward motion, and if r < 0 the target needs to be approached backing up. When no
 * information is specified, then the direction of approach is assumed to be forward.
 */
struct control_law_coordinates_t
{
    float r;
    float theta;
    float delta;

    control_law_coordinates_t(float r = 0, float theta = 0, float delta = 0) : r(r), theta(theta), delta(delta) { }

    control_law_coordinates_t(const pose_t& robotPose,
                              const pose_t& targetPose,
                              const approach_direction_t direction = FORWARD)
    {
        float deltaX = targetPose.x - robotPose.x;
        float deltaY = targetPose.y - robotPose.y;

        r = sqrt(deltaX * deltaX + deltaY * deltaY);
        theta = 0.0;
        delta = 0.0;

        // Fix to goal heading when the radius is small (~convergence radius), since
        // otherwise the line of sight can bacome singular at r = 0.
        // Also flip the robot and the target poses when trying to move backward.
        if (direction == FORWARD) {
            float lineOfSightOrientation = (r > SMALL_RADIUS_M) ? atan2(deltaY, deltaX) : targetPose.theta;
            theta = angle_diff(targetPose.theta, lineOfSightOrientation);
            delta = angle_diff(robotPose.theta, lineOfSightOrientation);
        } else if (direction
                   == BACKWARD)   // flip the target and the robot pose when the direction of approach is backward.
        {
            float lineOfSightOrientation =
              (r > SMALL_RADIUS_M) ? atan2(deltaY, deltaX) : angle_sum(targetPose.theta, M_PI);
            theta = angle_diff(angle_sum(targetPose.theta, M_PI), lineOfSightOrientation);
            delta = angle_diff(angle_sum(robotPose.theta, M_PI), lineOfSightOrientation);
            r *= -1;
        } else {
            std::cout << "ERROR: Unknown direction of approach!\n";
        }
    }

    pose_t toRobotPose(const pose_t& targetPose) const
    {
        float lineOfSightOrientation =
          (r > 0.0) ? angle_diff(targetPose.theta, theta) : angle_diff(angle_sum(targetPose.theta, M_PI), theta);
        float robotOrientation =
          (r > 0.0) ? angle_sum(lineOfSightOrientation, delta) : angle_sum(lineOfSightOrientation + M_PI, delta);

        return pose_t(targetPose.x - fabs(r) * cos(lineOfSightOrientation),
                      targetPose.y - fabs(r) * sin(lineOfSightOrientation),
                      robotOrientation);
    }

    pose_t toTargetPose(const pose_t& robotPose) const
    {
        float lineOfSightOrientation =
          (r > 0.0) ? angle_diff(robotPose.theta, delta) : angle_diff(angle_sum(robotPose.theta, M_PI), delta);
        float targetOrientation =
          (r > 0.0) ? angle_sum(lineOfSightOrientation, theta) : angle_sum(lineOfSightOrientation + M_PI, theta);

        return pose_t(robotPose.x + fabs(r) * cos(lineOfSightOrientation),
                      robotPose.y + fabs(r) * sin(lineOfSightOrientation),
                      targetOrientation);
    }
};


/**
 * motion_target_t encodes a target and variable parameters for the KinematicControlLaw, which is composed of:
 * (1) the target pose in reference frame,
 * (2) the direction with which to convege to the target pose, and
 * (3) the velocity gain to be used by the KinematicControlLaw.
 * (4) (optional) shape parameters (also can be interpreted as positional and steering gain parameters) for the
 * KinematicControlLaw
 *
 * NOTE: the sign of r in the control law coordinates contains the direction information.
 */
struct motion_target_t
{
    pose_t pose;                      // Target pose for the trajectory when it was simulated
    approach_direction_t direction;   // Direction with which to converge to the target
    float velocityGain;   // Velocity gain for the KinematicControlLaw, which is also a maximum allowed linear velocity.
    float k1;             // The first shape parameter (distance-to-heading gain)
    float k2;             // The second shape parameter (steering-to-error gain)

    motion_target_t(void) : pose(0, 0, 0), direction(FORWARD), velocityGain(0), k1(0), k2(0) { }

    motion_target_t(const pose_t& targetPose,
                    const approach_direction_t direction = FORWARD,
                    float velocityGain = DEFAULT_VELOCITY_GAIN)
    : pose(targetPose)
    , direction(direction)
    , velocityGain(velocityGain)
    , k1(DEFAULT_K1)
    , k2(DEFAULT_K2)
    {
    }

    motion_target_t(const control_law_coordinates_t& coords, const pose_t& robotPose, float velocityGain)
    : pose(coords.toTargetPose(robotPose))
    , direction((coords.r >= 0.0f) ? FORWARD : BACKWARD)
    , velocityGain(velocityGain)
    , k1(DEFAULT_K1)
    , k2(DEFAULT_K2)
    {
    }
    // NOTE: At the moment we only optimize over pose, direction, and velocity gain.
    //       As such, the motion controller will ignore any k1/k2 value in the received
    //       motion target. If any of these change, the constructor and communications
    //       needs to get expanded as well.
};

/**
 * control_law_output_t encodes the output of the KinematicControlLaw, which includes reference heading, curvature and
 * velocities as well as a indicator for reaching the target and the instance of control_law_coordinates_t used to
 * compute those values.
 */
struct control_law_output_t
{
    control_law_coordinates_t coords;

    float linearVelocity;
    float angularVelocity;

    float referenceHeading;
    float referenceKappa;

    bool haveReachedTarget;
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, motion_target_t& target)
{
    ar(target.pose, target.direction, target.velocityGain, target.k1, target.k2);
}

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_KINEMATIC_CONTROL_LAW_COORDINATES_H
