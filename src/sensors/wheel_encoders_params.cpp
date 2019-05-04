/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wheel_encoders_params.cpp
* \author   Collin Johnson
*
* Definition of parsers for the various params structs.
*/

#include <sensors/wheel_encoders_params.h>
#include <utils/config_file.h>

namespace vulcan
{
namespace sensors
{

const std::string ENCODERS_HEADING("WheelEncodersParameters");
const std::string TYPE_KEY        ("encoder_type");

const std::string ROBOT_CONFIG_HEADING("RobotEncoderConfiguration");
const std::string LEFT_CIRCUM_KEY ("left_wheel_circumference_m");
const std::string RIGHT_CIRCUM_KEY("right_wheel_circumference_m");
const std::string LEFT_TICKS_KEY  ("left_ticks_per_revolution");
const std::string RIGHT_TICKS_KEY ("right_ticks_per_revolution");
const std::string WHEELBASE_KEY   ("wheelbase_m");

const std::string PHIDGET_HEADING   ("PhidgetEncoderBoardParameters");
const std::string PHIDGET_SERIAL_KEY("serial_number");
const std::string PHIDGET_LEFT_KEY  ("left_encoder_index");
const std::string PHIDGET_RIGHT_KEY ("right_encoder_index");


robot_encoder_configuration_t  load_configuration_params(const utils::ConfigFile& config);
phidget_encoder_board_params_t load_phidget_board_params(const utils::ConfigFile& config);


wheel_encoders_params_t load_wheel_encoders_params(const utils::ConfigFile& config)
{
    wheel_encoders_params_t params;

    params.encoderType = config.getValueAsString(ENCODERS_HEADING, TYPE_KEY);

    params.configuration = load_configuration_params(config);
    params.phidgetParams = load_phidget_board_params(config);

    return params;
}


robot_encoder_configuration_t load_configuration_params(const utils::ConfigFile& config)
{
    robot_encoder_configuration_t robot;

    robot.leftWheelCircumference  = config.getValueAsFloat(ROBOT_CONFIG_HEADING, LEFT_CIRCUM_KEY);
    robot.rightWheelCircumference = config.getValueAsFloat(ROBOT_CONFIG_HEADING, RIGHT_CIRCUM_KEY);

    robot.leftTicksPerRevolution  = config.getValueAsInt32(ROBOT_CONFIG_HEADING, LEFT_TICKS_KEY);
    robot.rightTicksPerRevolution = config.getValueAsInt32(ROBOT_CONFIG_HEADING, RIGHT_TICKS_KEY);

    robot.wheelbase = config.getValueAsFloat(ROBOT_CONFIG_HEADING, WHEELBASE_KEY);

    return robot;
}


phidget_encoder_board_params_t load_phidget_board_params(const utils::ConfigFile& config)
{
    phidget_encoder_board_params_t params;

    params.serialNumber = config.getValueAsInt32(PHIDGET_HEADING, PHIDGET_SERIAL_KEY);
    params.leftIndex    = config.getValueAsInt32(PHIDGET_HEADING, PHIDGET_LEFT_KEY);
    params.rightIndex   = config.getValueAsInt32(PHIDGET_HEADING, PHIDGET_RIGHT_KEY);

    return params;
}

}
}
