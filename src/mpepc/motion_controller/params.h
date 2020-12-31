/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of the params structs for the motion controller module.
*/

#ifndef MPEPC_MOTION_CONTROLLER_PARAMS_H
#define MPEPC_MOTION_CONTROLLER_PARAMS_H

#include "mpepc/control/params.h"
#include "mpepc/motion_controller/waypoint_follower/params.h"
#include "robot/model/params.h"

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace mpepc
{

struct motion_target_following_controller_params_t
{
    kinematic_control_law_params_t            kinematicControlLawParams;
    joystick_control_law_params_t             joystickControlLawParams;
    robot::differential_motors_plant_params_t robotParams;
    
    motion_target_following_controller_params_t(const utils::ConfigFile& config, const utils::ConfigFile& robotConfig);
};


struct motion_controller_params_t
{
    std::vector<std::string> controllerTypes;
    
    motion_target_following_controller_params_t targetFollowerParams;
    waypoint_follower_params_t                  followerParams;
    graceful_motion_controller_params_t         gracefulParams;
    
    motion_controller_params_t(const utils::ConfigFile& config, const utils::ConfigFile& robotConfig);
};

/**
* load_motion_controller_params loads the set of parameters needed by the motion_controller.
*
* \param    config          Config file containing the parameters
* \return   Parameters structs read from the config file.
*/
motion_controller_params_t load_motion_controller_params(const utils::ConfigFile& config);


} // mpepc
} // vulcan

#endif // MPEPC_MOTION_CONTROLLER_PARAMS_H
