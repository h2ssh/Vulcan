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
* Declaration of the various params structs for the pieces of the robot_controller module.
*/

#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <string>
#include <math/geometry/rectangle.h>
#include <robot/wheelchair.h>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace robot
{

struct command_filter_params_t
{
    uint32_t sourceTimeout;
};

struct proximity_checker_params_t
{
    math::Rectangle<float> criticalBoundary;

    float warningRadius;
    float minSlowdownFactor;
    float maxSlowdownFactor;

    float lookaheadTime;
};

struct safe_stop_checker_params_t
{
    
};

struct motion_checker_params_t
{
    std::string checkerType;

    proximity_checker_params_t proximityParams;
    safe_stop_checker_params_t safeStopParams;
};

struct wheelchair_params_t
{
    std::string wheelchairType;
    std::string joystickPort;
    std::string controllerPort;

    wheelchair_joystick_calibration_t calibration;
};

struct robot_controller_params_t
{
    int64_t commandPeriodMs;

    command_filter_params_t filterParams;
    motion_checker_params_t checkerParams;
    wheelchair_params_t     driverParams;
};

/**
* load_command_period_ms loads the command period for the robot_interface from the provided config file.
*/
int load_command_period_ms(const utils::ConfigFile& config);

/**
* load_robot_controller_params loads the parameters for the wheelchair from the provided configuration
* file. The parameters are specified in the low_level_control_design_and_implementation document in the
* docs/design folder of the Vulcan repository.
*/
robot_controller_params_t load_robot_controller_params(const utils::ConfigFile& config);

}
}

#endif // ROBOT_WHEELCHAIR_CONTROLLER_PARAMS_H
