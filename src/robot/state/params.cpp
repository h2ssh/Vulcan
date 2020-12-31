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
 * Definition of load_state_estimator_module_params and associated functions.
 */

#include "robot/state/params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"

namespace vulcan
{
namespace robot
{

const std::string MODULE_HEADING("StateEstimatorModuleParameters");
const std::string MONITOR_TYPES_KEY("monitor_types");
const std::string UPDATE_PERIOD_KEY("update_period_ms");


int module_update_period_ms(const utils::ConfigFile& config)
{
    return config.getValueAsInt32(MODULE_HEADING, UPDATE_PERIOD_KEY);
}


state_estimator_module_params_t load_state_estimator_module_params(const utils::ConfigFile& config)
{
    state_estimator_module_params_t params;

    params.monitorTypes = utils::split_into_strings(config.getValueAsString(MODULE_HEADING, MONITOR_TYPES_KEY), ',');
    params.updatePeriodMs = config.getValueAsInt32(MODULE_HEADING, UPDATE_PERIOD_KEY);

    return params;
}

}   // namespace robot
}   // namespace vulcan
