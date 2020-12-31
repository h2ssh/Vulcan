/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_params.cpp
* \author   Zongtai Luo
*
* Definition of parsers for the various params structs.
*/

#include "simulator/simulator_params.h"

namespace vulcan
{
namespace sim
{

const std::string GENERAL_CONFIG_HEADING		("GeneralConfig");
const std::string MAP_PATH_KEY        			("map_path");
const std::string ROBOTS_NUM_KEY                ("robots_num");

const std::string MAIN_ROBOT_HEADING			("MainRobot");
const std::string FIRST_ROBOT_HEADING			("FirstRobot");
const std::string SECOND_ROBOT_HEADING			("SecondRobot");
const std::string THIRD_ROBOT_HEADING			("ThirdRobot");
const std::string ADD_ROBOT_HEADING             ("Robot");

const std::string IS_ACTIVATED_KEY				("is_activated");
const std::string ROBOT_DIMENSION_WIDTH         ("robot_rect_width");
const std::string ROBOT_DIMENSION_LENGTH        ("robot_rect_length");
const std::string SYSTEM_URL_KEY_KEY			("system_url");
const std::string SCRIPT_PATH_KEY				("script_path");
const std::string ROBOT_MODEL_KEY               ("robot_model");
const std::string ENCODER_MODEL_KEY             ("wheel_encoder_model");
const std::string CONTROLLER_TYPE_KEY           ("controller");


simulator_params_t load_simulator_params(const utils::ConfigFile& config, int case_no)
{
    simulator_params_t params;
    robot_params_t robot_params;

    params.map_path     = readingMaplpm(config, GENERAL_CONFIG_HEADING, MAP_PATH_KEY, case_no);
    params.robots_num   = config.getValueAsUInt8(GENERAL_CONFIG_HEADING, ROBOTS_NUM_KEY);
    
    robot_params  = load_robot_params(config, MAIN_ROBOT_HEADING,case_no);
    params.robot_params.push_back(robot_params);
    robot_params  = load_robot_params(config, FIRST_ROBOT_HEADING,case_no);
    params.robot_params.push_back(robot_params);
    robot_params  = load_robot_params(config, SECOND_ROBOT_HEADING,case_no);
    params.robot_params.push_back(robot_params);
    robot_params  = load_robot_params(config, THIRD_ROBOT_HEADING,case_no);
    params.robot_params.push_back(robot_params);

    return params;
}


robot_params_t load_robot_params(const utils::ConfigFile& config, const std::string& ROBOT_CONFIGURATION_HEADING, int case_no)
{
	robot_params_t params;

	if (config.getValueAsDouble(ROBOT_CONFIGURATION_HEADING, IS_ACTIVATED_KEY))
	{
        params.controller      = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, CONTROLLER_TYPE_KEY);
	    params.dimension.x     = config.getValueAsDouble(ROBOT_CONFIGURATION_HEADING, ROBOT_DIMENSION_WIDTH);
        params.dimension.y     = config.getValueAsDouble(ROBOT_CONFIGURATION_HEADING, ROBOT_DIMENSION_LENGTH);
	    params.system_url      = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, SYSTEM_URL_KEY_KEY);
        params.robot_model     = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, ROBOT_MODEL_KEY);
        params.encoder_model   = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, ENCODER_MODEL_KEY);
        if (case_no != 0)
        {
            params.script_path     = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, SCRIPT_PATH_KEY) + readingCaseSpt(ROBOT_CONFIGURATION_HEADING,case_no);
        }
        else
        {
            params.script_path     = config.getValueAsString(ROBOT_CONFIGURATION_HEADING, SCRIPT_PATH_KEY);
        }
	}
	
    return params;
}


std::string readingMaplpm(const utils::ConfigFile& config, const std::string& MAP_HEADING, const std::string& MAP_PATH_KEY, int case_no)
{
    std::string map_path;

    map_path = config.getValueAsString(MAP_HEADING, MAP_PATH_KEY);

    switch(case_no)
        {
            case 1:
                map_path += "Case_one.lpm";
                break;
            case 2:
                map_path += "Case_two.lpm";
                break;
            case 3:
                map_path += "Case_three.lpm";
                break;
        }

    return map_path;
}


std::string readingCaseSpt(const std::string HEADING, int case_no)
{   
    std::string case_file_name;

    switch(case_no)
        {
            case 1:
                case_file_name = "Case_one_";
                break;
            case 2:
                case_file_name = "Case_two_";
                break;
            case 3:
                case_file_name = "Case_three_";
                break;
        }

    if      (HEADING == "MainRobot")    {case_file_name += "main.spt";}
    else if (HEADING == "FirstRobot")   {case_file_name += "one.spt";}
    else if (HEADING == "SecondRobot")  {case_file_name += "two.spt";}
    else if (HEADING == "ThirdRobot")   {case_file_name += "three.spt";}
    
    return case_file_name;
}


robot_params_t load_new_robot_params(const utils::ConfigFile& config)
{
    return load_robot_params(config,ADD_ROBOT_HEADING,0);
}


} // sim
} // vulcan