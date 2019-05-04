/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     graceful_motion_control_law.cpp
* \author   Collin Johnson
*
* Definition of load_graceful_motion_params() and GracefulMotionControlLaw.
*/

#include <mpepc/motion_controller/waypoint_follower/graceful_motion_control_law.h>
#include <core/velocity.h>
#include <core/motion_state.h>
#include <utils/config_file.h>
#include <utils/timestamp.h>
#include <iostream>

// #define DEBUG_COMMAND
// #define DEBUG_TARGET
// #define DEBUG_VALIDATION

namespace vulcan
{

namespace mpepc
{

const std::string K1_KEY("k1");
const std::string K2_KEY("k2");

const std::string MAX_VEL_KEY("maximum_linear_velocity");
const std::string MAX_ANG_KEY("maximum_angular_velocity");

const std::string MAX_LIN_ACCEL_KEY("maximum_linear_acceleration");
const std::string MAX_ANG_ACCEL_KEY("maximum_angular_acceleration");

const std::string ANG_VEL_AT_TARGET_KEY("angular_velocity_at_target");
const std::string MIN_ANG_VEL_KEY      ("min_angular_velocity_at_target");
const std::string SLOWDOWN_RAD_KEY     ("slowdown_radius");
const std::string CONVERGE_RAD_KEY     ("convergence_radius");
const std::string CONVERGE_ANG_KEY     ("convergence_angle");
const std::string CONVERGE_TIME_KEY    ("convergence_time_ms");
const std::string BETA_KEY             ("beta");
const std::string LAMBDA_KEY           ("lambda");
const std::string VEL_GAIN_KEY         ("velocity_gain");

// graceful_motion_state_t maintains the necessary state for determining the velocities to command
struct graceful_motion_state_t
{
    graceful_motion_coordinates_t coords;

    float kappa;
    float radiusFreeKappa;

    float linearVelocity;
    float angularVelocity;

    float currentLinearVelocity;
    float currentAngularVelocity;

    float timestep;
};

// Helper functions for the calculation of the new command
void calculate_converged_radius_and_angle_velocities(graceful_motion_state_t& state, const graceful_motion_params_t& params);
void calculate_converged_radius_velocities          (graceful_motion_state_t& state, float deltaTheta, const graceful_motion_params_t& params);
void calculate_slowdown_radius_velocities           (graceful_motion_state_t& state, const graceful_motion_params_t& params);
void calculate_kappa_rule_velocities                (graceful_motion_state_t& state, const graceful_motion_params_t& params);

void calculate_kappa                         (graceful_motion_state_t& state, const graceful_motion_params_t& params);
void calculate_graceful_linear_velocity      (graceful_motion_state_t& state, const graceful_motion_params_t& params);
void calculate_graceful_angular_velocity     (graceful_motion_state_t& state, const graceful_motion_params_t& params);
void adjust_velocities_to_limit_accelerations(graceful_motion_state_t& state, const graceful_motion_params_t& params);
float calculate_velocity_decrease_ratio      (float commandedVelocity, float currentVelocity, float maxAcceleration);


graceful_motion_params_t load_graceful_motion_params(const std::string& heading, const utils::ConfigFile& config)
{
    graceful_motion_params_t params;

    params.k1 = config.getValueAsFloat(heading, K1_KEY);
    params.k2 = config.getValueAsFloat(heading, K2_KEY);

    params.maxLinearVelocity  = config.getValueAsFloat(heading, MAX_VEL_KEY);
    params.maxAngularVelocity = config.getValueAsFloat(heading, MAX_ANG_KEY);

    params.maxLinearAcceleration  = config.getValueAsFloat(heading, MAX_LIN_ACCEL_KEY);
    params.maxAngularAcceleration = config.getValueAsFloat(heading, MAX_ANG_ACCEL_KEY);

    params.angularVelocityAtTarget    = config.getValueAsFloat(heading, ANG_VEL_AT_TARGET_KEY);
    params.minAngularVelocityAtTarget = config.getValueAsFloat(heading, MIN_ANG_VEL_KEY);
    params.slowdownRadius             = config.getValueAsFloat(heading, SLOWDOWN_RAD_KEY);
    params.convergenceRadius          = config.getValueAsFloat(heading, CONVERGE_RAD_KEY);
    params.convergenceAngle           = config.getValueAsFloat(heading, CONVERGE_ANG_KEY);
    params.convergenceTime            = config.getValueAsInt32(heading, CONVERGE_TIME_KEY) * 1000ll;
    params.beta                       = config.getValueAsFloat(heading, BETA_KEY);
    params.lambda                     = config.getValueAsFloat(heading, LAMBDA_KEY);
    params.velocityGain               = config.getValueAsFloat(heading, VEL_GAIN_KEY);

    return params;
}


GracefulMotionControlLaw::GracefulMotionControlLaw(void)
    : haveTarget(false)
    , hasRadiusConverged(false)
    , hasAngleConverged(false)
    , hasTimeConverged(false)
{
}


void GracefulMotionControlLaw::reset(void)
{
    haveTarget = false;

    hasRadiusConverged = false;
    hasAngleConverged  = false;
    hasTimeConverged   = false;

    lastRadiusError      = HUGE_VALF;
    lastAngleError       = HUGE_VALF;
    convergenceStartTime = -1;
}


void GracefulMotionControlLaw::setTargetPose(const pose_t& pose, graceful_motion_direction_t direction, const graceful_motion_params_t& params)
{
    if(haveTarget && (target == pose))
    {
        return;
    }

    reset();

    haveTarget      = true;
    target          = pose;
    this->direction = direction;
    this->params    = params;

    if(direction == GRACEFUL_MOTION_BACKWARD)
    {
        target.theta = angle_sum(target.theta, M_PI);
    }

#ifdef DEBUG_TARGET
    std::cout<<"DEBUG:GracefulMotionControlLaw: Setting control target:"<<pose<<" Dir:"<<direction<<" Gain:"<<params.velocityGain<<'\n';
#endif
}


graceful_motion_output_t GracefulMotionControlLaw::apply(const motion_state_t& currentState, float timestep)
{
    return apply(currentState, timestep, params);
}


graceful_motion_output_t GracefulMotionControlLaw::apply(const motion_state_t& currentState, float timestep, const graceful_motion_params_t& params)
{
    graceful_motion_state_t state;

    // If not moving forward, then flip the pose to be used for calculating the controls
    float theta = (direction == GRACEFUL_MOTION_FORWARD) ? currentState.pose.theta : angle_sum(currentState.pose.theta, M_PI);
    pose_t currentPose{currentState.pose.x, currentState.pose.y, theta};

    float deltaTheta = angle_diff(target.theta, currentPose.theta);
    state.coords     = graceful_motion_coordinates_t(currentPose, target);

    state.timestep               = timestep;
    state.currentLinearVelocity  = currentState.velocity.linear;
    state.currentAngularVelocity = currentState.velocity.angular;

    graceful_motion_output_t output;

    output.haveReachedTarget = haveReachedTarget(currentPose);

    if((state.coords.r < params.convergenceRadius) && (std::abs(deltaTheta) < params.convergenceAngle))
    {
        calculate_converged_radius_and_angle_velocities(state, params);
    }
    else if(state.coords.r < params.convergenceRadius)
    {
        calculate_converged_radius_velocities(state, deltaTheta, params);
    }
    else if(state.coords.r < params.slowdownRadius)
    {
        calculate_slowdown_radius_velocities(state, params);
    }
    else // normal update
    {
        calculate_kappa_rule_velocities(state, params);
    }

    //     adjust_velocities_to_limit_accelerations(state, params);

    output.linearVelocity  = state.linearVelocity;
    output.angularVelocity = state.angularVelocity;
    output.kappa           = state.kappa;

    if(direction == GRACEFUL_MOTION_BACKWARD)
    {
        output.linearVelocity *= -1;
    }

#ifdef DEBUG_COMMAND
    std::cout<<"DEBUG:GracefulMotionControlLaw:current:"<<currentPose<<" target:"<<target
             <<" command:("<<output.linearVelocity<<','<<output.angularVelocity<<','<<output.kappa<<")\n";
#endif

    return output;
}


bool GracefulMotionControlLaw::haveReachedTarget(const pose_t& currentPose)
{
    // Reaching the target means both the distance to the target and the orientation offset have become small
    // numbers. To ensure the robot at least reaches the target, it'll overshoot a bit rather than not reach the target at all.

    float distance   = distance_between_points(currentPose.toPoint(), target.toPoint());
    float deltaTheta = angle_diff(target.theta, currentPose.theta);

    if(!hasRadiusConverged)
    {
        hasRadiusConverged = distance < params.convergenceRadius;
        lastRadiusError    = distance;
    }

    if(!hasAngleConverged)
    {
        hasAngleConverged = (std::abs(lastAngleError)  < params.convergenceAngle) &&
                            (std::abs(deltaTheta)      < params.convergenceAngle) &&
                            (lastAngleError*deltaTheta < 0);

        lastAngleError = deltaTheta;
    }

    // If the robot is in the convergence zone, then start the convergence time. This allows target convergence, even if
    // the robot can't quite reach a given target for some reason, thereby avoiding chatter with the brakes turning on/off
    if((std::abs(deltaTheta) < params.convergenceAngle) && (distance < params.convergenceRadius))
    {
        if(convergenceStartTime < 0)
        {
            convergenceStartTime = utils::system_time_us();
        }

        hasTimeConverged |= utils::system_time_us() - convergenceStartTime > params.convergenceTime;
    }
    else
    {
        convergenceStartTime = -1;
    }

#ifdef DEBUG_TARGET
    if(hasRadiusConverged && hasAngleConverged)
    {
        std::cout<<"GracefulMotionControlLaw: Converged to target:"<<target<<" Current:"<<currentPose<<'\n';
    }
    if(hasTimeConverged)
    {
        std::cout<<"GracefulMotionControlLaw: Converged for time:"<<target<<" Current:"<<currentPose<<'\n';
    }
#endif

    return (hasRadiusConverged && hasAngleConverged) || hasTimeConverged;
}

///////////////////////// Helpers for doing the actual control law calculations /////////////////////////////
void calculate_converged_radius_and_angle_velocities(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    // When converged, don't move!

    state.linearVelocity  = 0;
    state.angularVelocity = 0;
}


void calculate_converged_radius_velocities(graceful_motion_state_t& state, float deltaTheta, const graceful_motion_params_t& params)
{
    // If close enough, but not facing the right direction, then turn in place at a fixed
    // angular velocity

    // Use a proportional control where the maximum error is M_PI and decrease linearly until hitting the min velocity at 0 error
    // because the convergence doesn't happen at 0, the motion shouldn't creep to a halt as the error gets smaller
    float thetaError                  = std::abs(deltaTheta) / M_PI;
    float proportionalAngularVelocity = thetaError*(params.angularVelocityAtTarget - params.minAngularVelocityAtTarget) + params.minAngularVelocityAtTarget;

    state.linearVelocity  = 0;
    state.angularVelocity = copysign(proportionalAngularVelocity, deltaTheta);
}


void calculate_slowdown_radius_velocities(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    // For the slowdown, first calculate the normal kappa rule velocities. Then check to see if the
    // velocity due to the normal slowdown factor is less. Take the min of the two and then calculate the
    // angular velocity

    calculate_kappa_rule_velocities(state, params);

    float slowdownLinearVelocity = state.coords.r * params.velocityGain / params.slowdownRadius;

    if(slowdownLinearVelocity < state.linearVelocity)
    {
        state.linearVelocity = slowdownLinearVelocity;
        calculate_graceful_angular_velocity(state, params);
    }
}


void calculate_kappa_rule_velocities(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    calculate_kappa(state, params);
    calculate_graceful_linear_velocity(state, params);
    calculate_graceful_angular_velocity(state, params);
}


void calculate_kappa(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    /*
    * Kappa is the general control parameter. It is:
    *
    *   (-1/radius) * [k2*(delta - atan(-k1*theta)) + (1 + k1/(1+(k1*theta)^2))*sin(delta)]
    *
    * where k1 and k2 are parameters that adjust the balance between position and orientation following in the controller.
    */

    graceful_motion_coordinates_t& coords = state.coords;

    float k2Term  = params.k2 * (coords.delta - atan(-params.k1*coords.theta));
    float sinTerm = sin(coords.delta) * (1.0f + (params.k1 / (1.0f + pow(params.k1*coords.theta, 2))));

    state.radiusFreeKappa = -(k2Term + sinTerm);

    if(coords.r > params.convergenceRadius)
    {
        state.kappa = state.radiusFreeKappa / coords.r;
    }
    else
    {
        state.kappa = 0;
    }
}


void calculate_graceful_linear_velocity(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    /*
    * Linear velocity is a function of kappa. With a defined maximum velocity, the linear velocity scales with kappa
    * using the following formula:
    *
    *   v = v_max / (1 + beta*(abs(kappa)^lambda))
    *
    * where v_max is a system parameter and beta and lambda adjust the slowdown of the controller as it approaches its target.
    */

    state.linearVelocity = params.velocityGain / (1.0f + (params.beta * pow(std::abs(state.kappa), params.lambda)));

    if(state.linearVelocity > params.maxLinearVelocity)
    {
        state.linearVelocity = params.maxLinearVelocity;
    }
}


void calculate_graceful_angular_velocity(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    /*
    * Using the function for linear velocity that is based on kappa reduces the angular velocity to simply be
    * a fraction of the maximum linear velocity:
    *
    *   omega = kappa * v
    */

    state.angularVelocity = state.kappa * state.linearVelocity;

    if(state.angularVelocity > params.maxAngularVelocity)
    {
        state.angularVelocity = params.maxAngularVelocity;
        state.linearVelocity  = state.angularVelocity / state.kappa;
    }

    if(state.linearVelocity > params.maxLinearVelocity)
    {
        std::cerr<<"ERROR:graceful_motion:Unable to keep linear and angular velocity below maximum thresholds!:("
                 <<state.linearVelocity<<','<<state.angularVelocity<<") kappa:"<<state.kappa<<'\n';
    }
}


void adjust_velocities_to_limit_accelerations(graceful_motion_state_t& state, const graceful_motion_params_t& params)
{
    // If slamming the brakes, then ALWAYS allow that to be the case.
    if((state.linearVelocity == 0.0f) && (state.angularVelocity == 0.0f))
    {
        return;
    }

    float linearAcceleration  = state.linearVelocity  - state.currentLinearVelocity;
    float angularAcceleration = state.angularVelocity - state.currentAngularVelocity;

    // Multiplying by the timestep yields the maximum change in velocity for the timestep over which this command will be active
    float linearDecreaseRatio  = calculate_velocity_decrease_ratio(state.linearVelocity, state.currentAngularVelocity, params.maxLinearAcceleration*state.timestep);
    float angularDecreaseRatio = calculate_velocity_decrease_ratio(state.angularVelocity, state.currentAngularVelocity, params.maxAngularAcceleration*state.timestep);

    float decrease = (linearDecreaseRatio < angularDecreaseRatio) ? linearDecreaseRatio : angularDecreaseRatio;

    state.linearVelocity  = state.currentLinearVelocity  + linearAcceleration*decrease;
    state.angularVelocity = state.currentAngularVelocity + angularAcceleration*decrease;
}


float calculate_velocity_decrease_ratio(float commandedVelocity, float currentVelocity, float maxAcceleration)
{
    float ratio = 1.0f;

    float acceleration = std::abs(commandedVelocity - currentVelocity);

    if(acceleration > maxAcceleration)
    {
        ratio = maxAcceleration / acceleration;
    }

    return ratio;
}



} // namespace mpepc
} // namespace vulcan
