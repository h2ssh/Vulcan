/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     waypoint_follower.cpp
 * \author   Collin Johnson
 *
 * Definition of WaypointFollower.
 */

#include <mpepc/motion_controller/waypoint_follower/waypoint_follower.h>
#include <core/motion_state.h>
#include <robot/commands.h>
#include <core/angle_functions.h>
#include <utils/timestamp.h>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace mpepc
{

// Helper functions for attenuating the command signal
static float calculate_angular_velocity  (float kappa, float linearVelocity);
float        calculate_attenuation_factor(float attenuationPoint);
float        attenuate_value             (float attenuationFactor, float oldValue, float newValue);


WaypointFollower::WaypointFollower(const waypoint_follower_params_t& params)
    : haveTarget(false)
    , haveNextTarget(false)
    , attenuationStartTime(0)
    , params(params)
{
}


void WaypointFollower::setWaypointTarget(const controller_waypoint_t& target)
{
    if(!haveTarget)
    {
        this->target = target;
        haveTarget   = true;

        controlLaw.setTargetPose(target.pose, GRACEFUL_MOTION_FORWARD, params.gracefulMotionParams);

        waypointFollowerState = USE_KAPPA_VELOCITY;
    }
    else
    {
        nextTarget     = target;
        haveNextTarget = true;

        nextTargetControlLaw.setTargetPose(target.pose, GRACEFUL_MOTION_FORWARD, params.gracefulMotionParams);
    }
}


void WaypointFollower::pathCompleted(void)
{
    haveTarget     = false;
    haveNextTarget = false;
}


robot::velocity_command_t WaypointFollower::calculateVelocityCommand(const motion_state_t& currentState, float timestep)
{
    robot::velocity_command_t commandToIssue;

    if(!haveTarget)
    {
        return commandToIssue;
    }

    graceful_motion_coordinates_t coords(currentState.pose, target.pose);
    graceful_motion_output_t      velocities = controlLaw.apply(currentState, timestep);

    switch(waypointFollowerState)
    {
    case USE_KAPPA_VELOCITY:
        commandToIssue = calculateKappaLinearCommand(coords, velocities);

        if(coords.r < params.lookaheadDistance)
        {
            if(haveNextTarget)
            {
                waypointFollowerState = SLOWDOWN_LINEAR_VELOCITY;
            }
            else
            {
                waypointFollowerState = SLOWDOWN_TO_FINAL_TARGET;
            }
        }
        break;

    case SLOWDOWN_LINEAR_VELOCITY:
        commandToIssue = calculateSlowdownLinearCommand(coords, velocities);

        if(coords.r < target.radius)
        {
            finalCommandFromPreviousTarget = commandToIssue;
            target                         = nextTarget;
            attenuationStartTime           = utils::system_time_us();

            controlLaw.setTargetPose(nextTarget.pose, GRACEFUL_MOTION_FORWARD, params.gracefulMotionParams);

            haveNextTarget = false;

            waypointFollowerState = ATTENUATE_ANGULAR_VELOCITY;
        }
        break;

    case ATTENUATE_ANGULAR_VELOCITY:
        commandToIssue = calculateAttenuatedAngularCommand(coords, velocities);

        if(utils::system_time_us() - attenuationStartTime > target.attenuationTime)
        {
            attenuationStartTime  = utils::system_time_us();
            waypointFollowerState = SPEEDUP_TO_KAPPA_VELOCITY;
        }
        break;

    case SPEEDUP_TO_KAPPA_VELOCITY:
        commandToIssue = calculateSpeedupLinearCommand(coords, velocities);

        if(utils::system_time_us() - attenuationStartTime > params.speedupInterval)
        {
            waypointFollowerState = USE_KAPPA_VELOCITY;
        }
        break;

    case SLOWDOWN_TO_FINAL_TARGET:
        commandToIssue = calculateFinalTargetCommand(coords, velocities);

        if(haveNextTarget)
        {
            finalCommandFromPreviousTarget = commandToIssue;
            target                         = nextTarget;
            attenuationStartTime           = utils::system_time_us();

            controlLaw.setTargetPose(nextTarget.pose, GRACEFUL_MOTION_FORWARD, params.gracefulMotionParams);

            haveNextTarget = false;

            waypointFollowerState = ATTENUATE_ANGULAR_VELOCITY;
        }
        break;

    default:

        assert(false);
    }

    return commandToIssue;
}


robot::velocity_command_t WaypointFollower::calculateKappaLinearCommand(const graceful_motion_coordinates_t& coords, const graceful_motion_output_t& motion)
{
    robot::velocity_command_t newVelocityCommand;

    newVelocityCommand.linear  = motion.linearVelocity;
    newVelocityCommand.angular = motion.angularVelocity;

    return newVelocityCommand;
}


robot::velocity_command_t WaypointFollower::calculateSlowdownLinearCommand(const graceful_motion_coordinates_t& coords, const graceful_motion_output_t& motion)
{
    robot::velocity_command_t newVelocityCommand;

    graceful_motion_coordinates_t expectedCoords(target.pose, nextTarget.pose);
    graceful_motion_output_t      expectedVelocities = nextTargetControlLaw.apply(motion_state_t(target.pose, velocity_t()), 0.05f);
    float                         expectedKappa      = expectedVelocities.kappa;

    float residualKappa = -(params.gracefulMotionParams.k1 + params.gracefulMotionParams.k1*params.gracefulMotionParams.k1)*coords.theta / coords.r;

    float residualVelocity  = params.maxAngularVelocityChange / std::abs(residualKappa);
    float expectedVelocity  = params.maxAngularVelocityChange / std::abs(expectedKappa);
    float kappaDiffVelocity = params.maxAngularVelocityChange / std::abs(expectedKappa - residualKappa);

    float targetVelocity = (residualVelocity < expectedVelocity) ? residualVelocity  : expectedVelocity;
    targetVelocity       = (kappaDiffVelocity < targetVelocity)  ? kappaDiffVelocity : targetVelocity;

    float attenuationPoint  = (params.lookaheadDistance - coords.r) / (params.lookaheadDistance - target.radius);
    float attenuationFactor = calculate_attenuation_factor(attenuationPoint);

    newVelocityCommand.linear  = attenuate_value(attenuationFactor, motion.linearVelocity, targetVelocity);
    newVelocityCommand.angular = calculate_angular_velocity(newVelocityCommand.linear, motion.kappa);

    return newVelocityCommand;
}


robot::velocity_command_t WaypointFollower::calculateAttenuatedAngularCommand(const graceful_motion_coordinates_t& coords, const graceful_motion_output_t& motion)
{
    robot::velocity_command_t newVelocityCommand;

    int64_t currentTime = utils::system_time_us();

    newVelocityCommand.linear  = finalCommandFromPreviousTarget.linear;
    newVelocityCommand.angular = calculate_angular_velocity(motion.kappa, newVelocityCommand.linear);

    float attenuationPoint = static_cast<float>(currentTime - attenuationStartTime) / target.attenuationTime;
    float attenuationFactor = calculate_attenuation_factor(attenuationPoint);

    newVelocityCommand.angular = attenuate_value(attenuationFactor, finalCommandFromPreviousTarget.angular, newVelocityCommand.angular);

    return newVelocityCommand;
}


robot::velocity_command_t WaypointFollower::calculateSpeedupLinearCommand(const graceful_motion_coordinates_t& coords, const graceful_motion_output_t& motion)
{
    robot::velocity_command_t newVelocityCommand;

    int64_t currentTime = utils::system_time_us();

    float attenuationPoint  = static_cast<float>(currentTime - attenuationStartTime) / params.speedupInterval;
    float attenuationFactor = calculate_attenuation_factor(attenuationPoint);

    newVelocityCommand.linear  = attenuate_value(attenuationFactor, finalCommandFromPreviousTarget.linear, motion.linearVelocity);
    newVelocityCommand.angular = calculate_angular_velocity(motion.kappa, newVelocityCommand.linear);

    return newVelocityCommand;
}


robot::velocity_command_t WaypointFollower::calculateFinalTargetCommand(const graceful_motion_coordinates_t& coords, const graceful_motion_output_t& motion)
{
    robot::velocity_command_t newVelocityCommand;

    float attenuationPoint  = (params.lookaheadDistance - coords.r) / (params.lookaheadDistance - params.finalTargetStopRadius);
    float attenuationFactor = calculate_attenuation_factor(attenuationPoint);

    newVelocityCommand.linear  = attenuate_value(attenuationFactor, motion.linearVelocity, params.finalTargetVelocity);
    newVelocityCommand.angular = calculate_angular_velocity(newVelocityCommand.linear, motion.kappa);

    return newVelocityCommand;
}


float calculate_angular_velocity(float kappa, float linearVelocity)
{
    return linearVelocity * kappa;
}


// Helper functions for attenuating the command signal
float calculate_attenuation_factor(float attenuationPoint)
{
    /*
        * The attenuation factor controls the transition from the old target command to the new target command.
        * The attenuation is a modified sigmoid function with domain [0, duration] and range [0, 1]. The value
        * is calculated as:
        *
        *   sigma = (1/0.98) * (1 / (1 + exp(-9.2 * (attenuationPoint - 0.5))) - 0.01)
        */

    float attenuationFactor = 1.0f;  // if beyond the attenuation time, 1.0f means use the new command only

    // NOTE: This equation is pulled straight from the paper. I have no idea where these parameters came from.
    const float SCALE_FACTOR = 1.0f / 0.98f;

    float exponentialPower = -9.2 * (attenuationPoint - 0.5);

    attenuationFactor = SCALE_FACTOR * (1.0f/(1.0f + exp(exponentialPower)) - 0.01);

    return attenuationFactor;
}


float attenuate_value(float attenuationFactor, float oldValue, float newValue)
{
    return oldValue*(1.0f - attenuationFactor) + newValue*attenuationFactor;
}

} // namespace mpepc
} // namespace vulcan
