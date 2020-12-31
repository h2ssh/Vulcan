/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     state_estimator.cpp
 * \author   Collin Johnson
 *
 * Definition of create_state_estimator factory.
 */

#include "robot/state/state_estimator.h"
#include "robot/state/elevator_monitor.h"
#include "robot/state/motion_state_estimator.h"
#include "utils/config_file.h"
#include <cassert>
#include <iostream>
#include <memory>

namespace vulcan
{
namespace robot
{

const std::string MOTION_STATE_ESTIMATOR_HEADING("MotionStateEstimatorParameters");
const std::string ROBOT_MODEL_CONFIG_FILE_KEY("robot_model_config_file");

std::unique_ptr<StateEstimator> create_state_estimator(const std::string& type, const utils::ConfigFile& config)
{
    if (type == kElevatorMonitorType) {
        return std::unique_ptr<StateEstimator>(new ElevatorMonitor(elevator_monitor_params_t(config)));
    } else if (type == kMotionStateEstimatorType) {
        utils::ConfigFile robotConfig(
          config.getValueAsString(MOTION_STATE_ESTIMATOR_HEADING, ROBOT_MODEL_CONFIG_FILE_KEY));

        return std::unique_ptr<StateEstimator>(
          new MotionStateEstimator(motion_state_estimator_params_t(config, robotConfig)));
    }

    std::cerr << "ERROR: create_state_monitor: Unknown type: " << type << std::endl;
    assert(false);

    return std::unique_ptr<StateEstimator>();
}

}   // namespace robot
}   // namespace vulcan
