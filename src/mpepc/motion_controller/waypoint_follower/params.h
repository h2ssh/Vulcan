/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Declaration of the params structs for the path follower module.
*/

#ifndef MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_PARAMS_H
#define MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_PARAMS_H

#include <mpepc/motion_controller/waypoint_follower/graceful_motion_control_law.h>
#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace mpepc
{

struct graceful_motion_controller_params_t
{
    graceful_motion_params_t gracefulParams;
};

struct waypoint_follower_params_t
{
    // Parameters for calculating maximum linear velocity
    float   lookaheadDistance;
    float   angularAttenuationDistance;
    int64_t speedupInterval;
    
    float attenuationLinearVelocity;
    float maxAngularVelocityChange;
    float finalTargetVelocity;
    float finalTargetStopRadius;
    
    graceful_motion_params_t gracefulMotionParams;
};

waypoint_follower_params_t          load_waypoint_follower_params(const utils::ConfigFile& config);
graceful_motion_controller_params_t load_graceful_controller_params(const utils::ConfigFile& config);

} // mpepc
} // vulcan

#endif // MPEPC_MOTION_CONTROLLER_WAYPOINT_FOLLOWER_PARAMS_H
