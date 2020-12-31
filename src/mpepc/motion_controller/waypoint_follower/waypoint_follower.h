/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     waypoint_follower.h
 * \author   Collin Johnson
 *
 * Declaration of WaypointFollower interface and create_waypoint_follower factory.
 */

#ifndef MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_H
#define MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_H

#include "mpepc/motion_controller/task/path.h"
#include "mpepc/motion_controller/waypoint_follower/graceful_motion_control_law.h"
#include "mpepc/motion_controller/waypoint_follower/params.h"
#include "robot/commands.h"

namespace vulcan
{
struct motion_state_t;

namespace robot
{
struct velocity_command_t;
}

namespace mpepc
{

struct controller_waypoint_t;
struct waypoint_follower_params_t;

/**
 * WaypointFollower implements the waypoint following algorithm described in
 *
 * "A Smooth Control Law for Graceful Motion of a Mobile Robot" by Park and Kuipers, ICRA 2011.
 *
 * The graceful motion control law is used to calculate the velocity when moving to a waypoint.
 * Once within a certain range of a waypoint, the control is switched to the next waypoint with
 * some amount of attenuation between the control signals derivied from the previous waypoint.
 * This attenuation ensures graceful motion as control changes between different targets.
 */
class WaypointFollower
{
public:
    /**
     * Constructor for WaypointFollower.
     *
     * \param    params          Parameters to use for the following
     */
    WaypointFollower(const waypoint_follower_params_t& params);

    /**
     * setWaypointTarget sets the new target for the WaypointFollower to guide the robot towards.
     */
    void setWaypointTarget(const controller_waypoint_t& target);

    /**
     * pathCompleted tells the waypoint follower that the current path has been completed,
     * so it should go into the idle state.
     */
    void pathCompleted(void);

    /**
     * calculateVelocityCommand determines the motion command to issue based on the active target
     * and the current robot pose.
     */
    robot::velocity_command_t calculateVelocityCommand(const motion_state_t& currentState, float timestep);

private:
    robot::velocity_command_t calculateKappaLinearCommand(const graceful_motion_coordinates_t& coords,
                                                          const graceful_motion_output_t& motion);
    robot::velocity_command_t calculateSlowdownLinearCommand(const graceful_motion_coordinates_t& coords,
                                                             const graceful_motion_output_t& motion);
    robot::velocity_command_t calculateAttenuatedAngularCommand(const graceful_motion_coordinates_t& coords,
                                                                const graceful_motion_output_t& motion);
    robot::velocity_command_t calculateSpeedupLinearCommand(const graceful_motion_coordinates_t& coords,
                                                            const graceful_motion_output_t& motion);
    robot::velocity_command_t calculateFinalTargetCommand(const graceful_motion_coordinates_t& coords,
                                                          const graceful_motion_output_t& motion);

    enum control_state_t
    {
        USE_KAPPA_VELOCITY,
        SLOWDOWN_LINEAR_VELOCITY,
        ATTENUATE_ANGULAR_VELOCITY,
        SPEEDUP_TO_KAPPA_VELOCITY,
        SLOWDOWN_TO_FINAL_TARGET
    };

    control_state_t waypointFollowerState;

    controller_waypoint_t target;
    bool haveTarget;

    controller_waypoint_t nextTarget;
    bool haveNextTarget;

    robot::velocity_command_t mostRecentCommand;
    robot::velocity_command_t finalCommandFromPreviousTarget;   // When the target changes, need to previous command
    // so to allow for smooth changeover of motion
    int64_t attenuationStartTime;

    GracefulMotionControlLaw controlLaw;
    GracefulMotionControlLaw nextTargetControlLaw;

    waypoint_follower_params_t params;
};


}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_H
