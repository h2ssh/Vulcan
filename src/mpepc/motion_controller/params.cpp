/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.cpp
 * \author   Collin Johnson and Jong Jin Parks
 *
 * Definition of parsers for the various parameters structs for the motion_controller.
 */

#include "mpepc/motion_controller/params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"

namespace vulcan
{

namespace mpepc
{

const std::string kMotionControllerHeading("MotionControllerDirectorParameters");
const std::string kControllerTypesKey("controller_types");
const std::string kRobotModelConfigFileKey("robot_model_config_file");

motion_target_following_controller_params_t::motion_target_following_controller_params_t(
  const utils::ConfigFile& config,
  const utils::ConfigFile& robotConfig)
: kinematicControlLawParams(config)
, joystickControlLawParams(config)
, robotParams(robotConfig)
{
}

motion_controller_params_t::motion_controller_params_t(const utils::ConfigFile& config,
                                                       const utils::ConfigFile& robotConfig)
: controllerTypes(
  utils::split_into_strings(config.getValueAsString(kMotionControllerHeading, kControllerTypesKey), ','))
, targetFollowerParams(config, robotConfig)
{
}

motion_controller_params_t load_motion_controller_params(const utils::ConfigFile& config)
{

    utils::ConfigFile robotConfig(config.getValueAsString(kMotionControllerHeading, kRobotModelConfigFileKey));
    motion_controller_params_t params(config, robotConfig);

    params.followerParams = load_waypoint_follower_params(config);
    params.gracefulParams = load_graceful_controller_params(config);

    return params;
}


}   // namespace mpepc
}   // namespace vulcan
