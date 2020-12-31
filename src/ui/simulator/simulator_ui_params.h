/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     simulator_ui_params.h
 * \author   Zongtai Luo
 *
 * Reading system url and main script.
 */

#ifndef UI_SIMULATOR_SIMULATOR_UI_PARAMS_H
#define UI_SIMULATOR_SIMULATOR_UI_PARAMS_H

#include "utils/config_file.h"

namespace vulcan
{
namespace ui
{

struct robot_receiver_params_t
{
    int8_t robot_num;
    std::vector<std::string> system_urls;
    std::string main_script_path;
};


std::string load_system_url(const utils::ConfigFile& config, const std::string& ROBOT_CONFIGURATION_HEADING = "Robot");


robot_receiver_params_t load_simulator_ui_params(const utils::ConfigFile& config);


}   // namespace ui
}   // namespace vulcan

#endif