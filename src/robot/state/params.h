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
 * Declaration of params structs for the state_monitor module and the load_state_monitor_module_params() function.
 */

#ifndef ROBOT_STATE_PARAMS_H
#define ROBOT_STATE_PARAMS_H

#include <string>
#include <vector>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace robot
{

struct state_estimator_module_params_t
{
    std::vector<std::string> monitorTypes;
    int updatePeriodMs;
};

/**
 * module_update_period_ms pulls the update period from the ConfigFile. It needs to be loaded before the parameters
 * are initialized, so it loaded separately from the rest of the params.
 *
 * \param    config              ConfigFile controlling the module parameters
 * \return   Update period for the module in milliseconds.
 */
int module_update_period_ms(const utils::ConfigFile& config);

/**
 * load_state_monitor_module_params loads the module parameters from the provided configuration file.
 *
 * \param    config          Configuration file with the parameters for the module
 * \return   Parameters pulled from the config file.
 */
state_estimator_module_params_t load_state_estimator_module_params(const utils::ConfigFile& config);

}   // namespace robot
}   // namespace vulcan

#endif   // ROBOT_STATE_PARAMS_H
