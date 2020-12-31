/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.h
 * \author   Jong Jin Park
 *
 * Declaration of params structs for DynamicObjectSimulator.
 */

#ifndef MPEPC_DYNAMIC_OBJECT_SIMULATOR_PARAMS_H
#define MPEPC_DYNAMIC_OBJECT_SIMULATOR_PARAMS_H

#include "mpepc/control/params.h"
#include "mpepc/motion_controller/params.h"
#include "robot/model/params.h"

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace mpepc
{

struct robot_simulator_params_t
{
    // controller model parameters
    kinematic_control_law_params_t kinematicControlLawParams;
    joystick_control_law_params_t joystickControlLawParams;

    // plant model paramsters
    robot::plant_model_params_t robotPlantModelParams;

    robot_simulator_params_t(void){};
    robot_simulator_params_t(const utils::ConfigFile& controllerConfig, const utils::ConfigFile& robotConfig);
};

struct dynamic_object_filter_params_t
{
    int64_t staleObjectTimeUs;

    float maxObjectSpeed;   // cap all object speeds at this maximum
                            // helps avoid problems with bad initial velocity estimates
    double maxTrustedVelocityStd;
    double startUntrustedVelocityStd;

    double minGoalProbability;   // minimum probability to use a goal vs. velocity estimate for prediction

    bool shouldSlowdownObjectsBehindRobot;
    float slowdownObjectConeAngle;
    float ignoreObjectConeRadius;

    dynamic_object_filter_params_t(void){};
    dynamic_object_filter_params_t(const utils::ConfigFile& config);
};

struct dynamic_object_simulator_params_t
{
    bool shouldPredictObjectVelocities;
    float lookaheadTime;
    float reactionTime;

    dynamic_object_simulator_params_t(void){};
    dynamic_object_simulator_params_t(const utils::ConfigFile& config);
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_DYNAMIC_OBJECT_SIMULATOR_PARAMS_H
