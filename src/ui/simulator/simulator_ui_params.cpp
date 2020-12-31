/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_ui_params.cpp
* \author   Zongtai Luo
*
* Reading system url and main script.
*/

#include "ui/simulator/simulator_ui_params.h"

namespace vulcan
{
namespace ui
{

const std::string GENERAL_CONFIG_HEADING        ("GeneralConfig");
const std::string ROBOTS_NUM_KEY                ("robots_num");

const std::string MAIN_ROBOT_HEADING			("MainRobot");
const std::string FIRST_ROBOT_HEADING			("FirstRobot");
const std::string SECOND_ROBOT_HEADING          ("SecondRobot");
const std::string ADD_ROBOT_HEADING             ("Robot");

const std::string SYSTEM_URL_KEY_KEY			("system_url");
const std::string SCRIPT_PATH_KEY               ("script_path");

std::string load_system_url(const utils::ConfigFile& config, const std::string& ROBOT_CONFIGURATION_HEADING)
{
    std::string system_url;

    system_url = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, SYSTEM_URL_KEY_KEY);
    
    return system_url;
}


robot_receiver_params_t load_simulator_ui_params(const utils::ConfigFile& config)
{
    robot_receiver_params_t params;
    std::string system_url;
    
    params.robot_num   = config.getValueAsUInt8(GENERAL_CONFIG_HEADING, ROBOTS_NUM_KEY);

    system_url  = load_system_url(config, MAIN_ROBOT_HEADING);
    params.system_urls.push_back(system_url);
    system_url  = load_system_url(config, FIRST_ROBOT_HEADING);
    params.system_urls.push_back(system_url);
    system_url  = load_system_url(config, SECOND_ROBOT_HEADING);
    params.system_urls.push_back(system_url);

    params.main_script_path = config.getValueAsString(MAIN_ROBOT_HEADING, SCRIPT_PATH_KEY);

    return params;
}


} // sim
} // vulcan
