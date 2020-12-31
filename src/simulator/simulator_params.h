/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     simulator_params.h
 * \author   Zongtai Luo
 *
 * Declaration of params structs for the classes dealing with simulator.
 */

#ifndef ENVIRONMENT_SIMULATOR_SIMULATOR_PARAMS_H
#define ENVIRONMENT_SIMULATOR_SIMULATOR_PARAMS_H

#include "core/point.h"
#include "core/pose.h"
#include "utils/config_file.h"
#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace sim
{

struct robot_params_t
{
    std::string controller;

    Point<double> dimension;

    std::string robot_model;
    std::string encoder_model;
    std::string system_url;
    std::string script_path;

    robot_params_t(void) : system_url(""), script_path("") { }

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(controller, dimension, robot_model, encoder_model, system_url, script_path);
    }
};

struct simulator_params_t
{
    std::string map_path;

    int8_t robots_num;

    std::vector<robot_params_t> robot_params;

    simulator_params_t(void) : map_path(""), robots_num(0), robot_params(){};
};

simulator_params_t load_simulator_params(const utils::ConfigFile& config, int case_no = 1);

robot_params_t
  load_robot_params(const utils::ConfigFile& config, const std::string& ROBOT_CONFIGURATION_HEADING, int case_no);

std::string readingMaplpm(const utils::ConfigFile& config,
                          const std::string& MAP_HEADING,
                          const std::string& MAP_KEY,
                          int case_no);

std::string readingCaseSpt(const std::string HEADING, int case_no);

robot_params_t load_new_robot_params(const utils::ConfigFile& config);

}   // namespace sim
}   // namespace vulcan

#endif   // ENVIRONMENT_SIMULATOR_SIMULATOR_PARAMS_H