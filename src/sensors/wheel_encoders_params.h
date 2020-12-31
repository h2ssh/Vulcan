/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     wheel_encoders_params.h
 * \author   Collin Johnson
 *
 * Declaration of params structs for the classes dealing with encoders and odometry.
 */

#ifndef SENSORS_WHEEL_ENCODERS_PARAMS_H
#define SENSORS_WHEEL_ENCODERS_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace sensors
{

struct phidget_encoder_board_params_t
{
    int serialNumber;
    int leftIndex;
    int rightIndex;
};

struct robot_encoder_configuration_t
{
    float leftWheelCircumference;
    float rightWheelCircumference;

    int leftTicksPerRevolution;
    int rightTicksPerRevolution;

    float wheelbase;
};

struct wheel_encoders_params_t
{
    std::string encoderType;

    robot_encoder_configuration_t configuration;
    phidget_encoder_board_params_t phidgetParams;
};

/**
 * load_wheel_encoders_params loads the parameters for the wheel encoders drivers.
 *
 * \param    config          ConfigFile containing the parameters
 * \return   Struct parsed from the config file.
 */
wheel_encoders_params_t load_wheel_encoders_params(const utils::ConfigFile& config);

}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_WHEEL_ENCODERS_PARAMS_H
