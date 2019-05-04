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
* Definition of the parsers for the various params structs for the pieces of the robot_controller module.
*/

#include <robot/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>

namespace vulcan
{
namespace robot
{

// Subscription channel keys
const std::string CONTROLLER_HEADING("RobotInterfaceParameters");
const std::string PERIOD_KEY("command_period_ms");

// command filter keys
const std::string FILTER_HEADING("VelocityCommandFilterParameters");
const std::string SOURCE_TIMEOUT_KEY("source_timeout_ms");

// motion checker keys
const std::string CHECKER_HEADING("MotionCheckerParameters");
const std::string CHECKER_TYPE_KEY("motion_checker_type");

// proximity checker keys
const std::string PROXIMITY_HEADING("ProximityCheckerParameters");
const std::string CRITICAL_BOUND_KEY("critical_bound");
const std::string WARNING_RADIUS_KEY("warning_radius_m");
const std::string MIN_SLOWDOWN_KEY("min_slowdown_factor");
const std::string MAX_SLOWDOWN_KEY("max_slowdown_factor");
const std::string LOOKAHEAD_KEY("lookahead_time_ms");

const std::string SAFE_STOP_HEADING("SafeStopCheckerParameters");

// wheelchair keys
const std::string WHEELCHAIR_HEADING("WheelchairParameters");
const std::string WHEELCHAIR_TYPE_KEY("wheelchair_type");
const std::string JOYSTICK_PORT_KEY("joystick_port");
const std::string CONTROLLER_PORT_KEY("controller_port");
const std::string POSITIVE_LINEAR_VEL_KEY("positive_linear_velocity_ratio");
const std::string NEGATIVE_LINEAR_VEL_KEY("negative_linear_velocity_ratio");
const std::string TRANS_POSITIVE_ANG_VEL_KEY("translating_positive_angular_velocity_ratio");
const std::string TRANS_NEGATIVE_ANG_VEL_KEY("translating_negative_angular_velocity_ratio");
const std::string TURN_POSITIVE_ANG_VEL_KEY("turn_in_place_positive_angular_velocity_ratio");
const std::string TURN_NEGATIVE_ANG_VEL_KEY("turn_in_place_negative_angular_velocity_ratio");


motion_checker_params_t           load_motion_checker_params   (const utils::ConfigFile& config);
command_filter_params_t           load_command_filter_params   (const utils::ConfigFile& config);
proximity_checker_params_t        load_proximity_checker_params(const utils::ConfigFile& config);
safe_stop_checker_params_t        load_safe_stop_checker_params(const utils::ConfigFile& config);
wheelchair_params_t               load_wheelchair_params       (const utils::ConfigFile& config);
wheelchair_joystick_calibration_t load_joystick_calibration    (const utils::ConfigFile& config);


int load_command_period_ms(const utils::ConfigFile& config)
{
    return config.getValueAsInt32(CONTROLLER_HEADING, PERIOD_KEY);
}


robot_controller_params_t load_robot_controller_params(const utils::ConfigFile& config)
{
    robot_controller_params_t params;

    params.commandPeriodMs = config.getValueAsInt32(CONTROLLER_HEADING, PERIOD_KEY);

    params.filterParams  = load_command_filter_params(config);
    params.checkerParams = load_motion_checker_params(config);
    params.driverParams  = load_wheelchair_params(config);

    return params;
}


command_filter_params_t load_command_filter_params(const utils::ConfigFile& config)
{
    command_filter_params_t params;

    params.sourceTimeout = config.getValueAsUInt32(FILTER_HEADING, SOURCE_TIMEOUT_KEY) * 1000;

    return params;
}


motion_checker_params_t load_motion_checker_params(const utils::ConfigFile& config)
{
    motion_checker_params_t params;

    params.checkerType     = config.getValueAsString(CHECKER_HEADING, CHECKER_TYPE_KEY);
    params.proximityParams = load_proximity_checker_params(config);
    params.safeStopParams  = load_safe_stop_checker_params(config);

    return params;
}


proximity_checker_params_t load_proximity_checker_params(const utils::ConfigFile& config)
{
    proximity_checker_params_t params;

    params.criticalBoundary = utils::create_rectangle_from_string(config.getValueAsString(PROXIMITY_HEADING, CRITICAL_BOUND_KEY));

    params.warningRadius     = config.getValueAsFloat(PROXIMITY_HEADING, WARNING_RADIUS_KEY);
    params.minSlowdownFactor = config.getValueAsFloat(PROXIMITY_HEADING, MIN_SLOWDOWN_KEY);
    params.maxSlowdownFactor = config.getValueAsFloat(PROXIMITY_HEADING, MAX_SLOWDOWN_KEY);

    params.lookaheadTime = config.getValueAsUInt16(PROXIMITY_HEADING, LOOKAHEAD_KEY) / 1000.0f;

    return params;
}


safe_stop_checker_params_t load_safe_stop_checker_params(const utils::ConfigFile& config)
{
    safe_stop_checker_params_t params;
    
    return params;
}


wheelchair_params_t load_wheelchair_params(const utils::ConfigFile& config)
{
    wheelchair_params_t params;

    params.wheelchairType = config.getValueAsString(WHEELCHAIR_HEADING, WHEELCHAIR_TYPE_KEY);
    params.joystickPort   = config.getValueAsString(WHEELCHAIR_HEADING, JOYSTICK_PORT_KEY);
    params.controllerPort = config.getValueAsString(WHEELCHAIR_HEADING, CONTROLLER_PORT_KEY);
    params.calibration    = load_joystick_calibration(config);

    return params;
}


wheelchair_joystick_calibration_t load_joystick_calibration(const utils::ConfigFile& config)
{
    wheelchair_joystick_calibration_t calibration;

    calibration.positiveLinearVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, POSITIVE_LINEAR_VEL_KEY);
    calibration.negativeLinearVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, NEGATIVE_LINEAR_VEL_KEY);

    calibration.translatingPositiveAngularVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, TRANS_POSITIVE_ANG_VEL_KEY);
    calibration.translatingNegativeAngularVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, TRANS_NEGATIVE_ANG_VEL_KEY);

    calibration.turnInPlacePositiveAngularVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, TURN_POSITIVE_ANG_VEL_KEY);
    calibration.turnInPlaceNegativeAngularVelocityRatio = config.getValueAsDouble(WHEELCHAIR_HEADING, TURN_NEGATIVE_ANG_VEL_KEY);

    return calibration;
}

} // namespace robot
} // namespace vulcan
