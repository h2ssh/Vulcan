/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     graceful_motion_control_law.h
* \author   Collin Johnson
*
* Declaration of GracefulMotionControlLaw, along with graceful_motion_coordinates_t
* and graceful_motion_params_t.
*/

#ifndef MPEPC_GRACEFUL_MOTION_CONTROL_LAW_H
#define MPEPC_GRACEFUL_MOTION_CONTROL_LAW_H

#include "core/angle_functions.h"
#include "core/pose.h"
#include <string>

namespace vulcan
{
struct motion_state_t;

namespace utils { class ConfigFile;  }

namespace mpepc
{

/**
* graceful_motion_coordinates_t defines the set of coordinates for describing the current control
* law. The coordinate system is based on the polar coordinates of the difference between the
* robot pose and the waypoint target.
*
* The control law coordinates are:
*
*   r     : distance from current pose to the target
*   theta : angular difference between the line-of-sight line from robot to waypoint position and the waypoint target orientation
*   delta : angular difference between the line-of-sight line from robot to waypoint position and the robot orientation
*/
struct graceful_motion_coordinates_t
{
    float r;
    float theta;
    float delta;

    graceful_motion_coordinates_t(float r = 0, float theta = 0, float delta = 0)
        : r(r)
        , theta(theta)
        , delta(delta)
    {
    }

    graceful_motion_coordinates_t(const pose_t& current, const pose_t& target)
    {
        float deltaX                 = target.x - current.x;
        float deltaY                 = target.y - current.y;
        float lineOfSightOrientation = atan2(deltaY, deltaX);

        r     = sqrt(deltaX*deltaX + deltaY*deltaY);
        theta = angle_diff(target.theta,  lineOfSightOrientation);
        delta = angle_diff(current.theta, lineOfSightOrientation);
    }

    /**
    * toPose converts the graceful motion coordinates into a pose relative to the provided
    * reference pose.
    */
    pose_t toPose(const pose_t& reference) const
    {
        float orientation = reference.theta - delta;

        return pose_t(reference.x + r*cos(orientation),
                             reference.y + r*sin(orientation),
                             angle_sum(orientation, theta));

    }
};

/**
* graceful_motion_params_t defines the parameters to use for determining velocities with the graceful motion
* control law.
*
* Config file keys for params:
*
*   k1                         = "k1"
*   k2                         = "k2"
*   maxLinearVelocity          = "maximum_linear_velocity"
*   maxAngularVelocity         = "maximum_angular_velocity"
*   maxLinearAcceleration      = "maximum_linear_acceleration"
*   maxAngularAcceleration     = "maximum_angular_acceleration"
*   angularVelocityAtTarget    = "angular_velocity_at_target"
*   minAngularVelocityAtTarget = "min_angular_velocity_at_target"
*   slowdownRadius             = "slowdown_radius"
*   convergenceRadius          = "convergence_radius"
*   convergenceAngle           = "convergence_angle"
*   convergenceTime            = "converge_time_ms"
*   beta                       = "beta"
*   lambda                     = "lambda"
*   velocityGain               = "velocity_gain"
*/
struct graceful_motion_params_t
{
    // Parameters for calculating kappa
    float k1;
    float k2;

    // Parameters for limiting velocities -- keep them safe
    float maxLinearVelocity;
    float maxAngularVelocity;

    // Parameters for limiting acceleration -- keep them from being afraid
    float maxLinearAcceleration;
    float maxAngularAcceleration;

    // Parameters for controlling the angular velocity
    float angularVelocityAtTarget;
    float minAngularVelocityAtTarget;

    // Parameters determining behavior of the robot near the target
    float   slowdownRadius;
    float   convergenceRadius;
    float   convergenceAngle;
    int64_t convergenceTime;
    float   beta;
    float   lambda;

    // radius dependent linear velocity selection
    float velocityGain;
};

/**
* graceful_motion_output_t defines the output of an application of the control law. The angular and linear velocity and the
* kappa term relating the two quantities are available.
*
* If some other method for calculating the linear velocity is desired, the kappa value is independent of the actual velocity,
* and the relation between linear and angular velocity is: angular = kappa * linear.
*/
struct graceful_motion_output_t
{
    float linearVelocity;
    float angularVelocity;

    float kappa;

    bool  haveReachedTarget;  ///< Flag indicating if the target has been reached, hence commands are 0.
};

/**
* graceful_motion_direction_t specifies which direction the control should be run for the current target.
*/
enum graceful_motion_direction_t
{
    GRACEFUL_MOTION_FORWARD,
    GRACEFUL_MOTION_BACKWARD
};

/**
* load_graceful_motion_params loads the parameters for the graceful motion control law from the provided configuration file.
*
* \param    heading             Heading under which the graceful motion params keys will be found
* \param    config              ConfigFile in which the values exist
* \return   Parameters for the control law.
*/
graceful_motion_params_t load_graceful_motion_params(const std::string& heading, const utils::ConfigFile& config);

/**
* GracefulMotionControlLaw
*/
class GracefulMotionControlLaw
{
public:

    /**
    * Default constructor for GracefulMotionControlLaw.
    */
    GracefulMotionControlLaw(void);

    /**
    * reset resets the internal state of the control law to be as if no target is assigned, thereby ensuring that
    * setTargetPose results in a new target being specified.
    */
    void reset(void);

    /**
    * setTargetPose sets the target pose for the control law. If the target pose is the same as the previous target,
    * then target is not updated.
    *
    * \param    pose            Pose to which the robot should be driving
    * \param    direction       Direction in which the control law should run
    * \param    params          Control law parameters to use for driving to this target
    */
    void setTargetPose(const pose_t& pose, graceful_motion_direction_t direction, const graceful_motion_params_t& params);

    /**
    * apply applies the control law for motion to the current target pose. The expected linear and angular velocities
    * are returned.
    *
    * \param    currentState            The current state of the robot
    * \param    timestep                Amount of time between command steps
    * \return   Velocities to send to the robot
    */
    graceful_motion_output_t apply(const motion_state_t& currentState, float timestep);

    /**
    * apply applies the control law for motion to the current target pose. The expected linear and angular velocities
    * are returned.
    *
    * \param    currentState            The current state of the robot
    * \param    timestep                Amount of time between command steps
    * \param    params          Control law parameters to use for driving to this target
    * \return   Velocities to send to the robot
    */
    graceful_motion_output_t apply(const motion_state_t& currentState, float timestep, const graceful_motion_params_t& params);

    /**
    * haveReachedTarget checks to see if the assigned target has been reached.
    *
    * \param    currentPose             Current pose of the robot
    * \return   True if the target has been reached.
    */
    bool haveReachedTarget(const pose_t& currentPose);

private:

    bool                        haveTarget;
    pose_t               target;
    graceful_motion_direction_t direction;

    bool hasRadiusConverged;
    bool hasAngleConverged;
    bool hasTimeConverged;

    float   lastRadiusError;
    float   lastAngleError;
    int64_t convergenceStartTime;

    graceful_motion_params_t params;

};


} // mpepc
} // vulcan

#endif // MPEPC_GRACEFUL_MOTION_CONTROL_LAW_H
