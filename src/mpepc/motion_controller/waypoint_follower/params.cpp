/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Collin Johnson
*
* Definition of parsers for the various parameters structs for the motion_controller.
*/

#include <mpepc/motion_controller/waypoint_follower/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>

namespace vulcan
{

namespace mpepc
{

const std::string WAYPOINT_FOLLOWER_HEADING("WaypointFollowerParameters");
const std::string LOOKAHEAD_KEY            ("lookahead_distance");
const std::string ANGULAR_DISTANCE_KEY     ("angular_attenuation_distance");
const std::string SPEEDUP_KEY              ("speedup_interval");
const std::string ATTENUATION_VELOCITY_KEY ("attenuation_linear_velocity");
const std::string MAX_ANGULAR_CHANGE_KEY   ("max_angular_velocity_change");
const std::string FINAL_VELOCITY_KEY       ("final_target_velocity");
const std::string FINAL_TARGET_RADIUS_KEY  ("final_target_stop_radius");

const std::string GRACEFUL_CONTROLLER_HEADING("GracefulMotionControllerParameters");
// All parameters are those used in graceful_motion_params_t

waypoint_follower_params_t load_waypoint_follower_params(const utils::ConfigFile& config)
{
    waypoint_follower_params_t params;

    params.lookaheadDistance          = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, LOOKAHEAD_KEY);
    params.angularAttenuationDistance = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, ANGULAR_DISTANCE_KEY);
    params.speedupInterval            = config.getValueAsInt32(WAYPOINT_FOLLOWER_HEADING, SPEEDUP_KEY) * 1000;
    params.attenuationLinearVelocity  = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, ATTENUATION_VELOCITY_KEY);
    params.maxAngularVelocityChange   = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, MAX_ANGULAR_CHANGE_KEY);
    params.finalTargetVelocity        = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, FINAL_VELOCITY_KEY);
    params.finalTargetStopRadius      = config.getValueAsFloat(WAYPOINT_FOLLOWER_HEADING, FINAL_TARGET_RADIUS_KEY);
    
    params.gracefulMotionParams = load_graceful_motion_params(WAYPOINT_FOLLOWER_HEADING, config);

    return params;
}


graceful_motion_controller_params_t load_graceful_controller_params(const utils::ConfigFile& config)
{
    graceful_motion_controller_params_t params;

    params.gracefulParams = load_graceful_motion_params(GRACEFUL_CONTROLLER_HEADING, config);

    return params;
}

} // namespace planner
} // namespace vulcan
