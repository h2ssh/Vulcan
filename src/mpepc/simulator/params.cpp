/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Jong Jin Park
*
* Definition of the DynamicObjectSimulator parameters.
*/

#include <mpepc/simulator/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>
#include <cassert>

namespace vulcan
{
namespace mpepc
{

const std::string kDynamicObjectFilterHeading("DynamicObjectFilterParameters");
const std::string kStaleTimeKey("stale_object_time_ms");
const std::string kMaxObjectSpeedKey("max_object_speed");
const std::string kSmallObjectsAreStationaryKey("should_ignore_small_object_speed");
const std::string kMaxVelocityStdKey("max_trusted_velocity_std_dev");
const std::string kStartUntrustedVelocityStdKey("start_untrusted_velocity_std_dev");
const std::string kMinGoalProbabilityKey("min_goal_probability");
const std::string kShouldSlowdownObjectBehindRobotKey("should_slowdown_objects_behind_robot");
const std::string kSlowdownConeAngleKey("slowdown_cone_width_degrees");
const std::string kBlindConeRadiusKey("blind_cone_radius_m");

const std::string kDynamicObjectSimlatorHeading("DynamicObjectSimulatorParameters");
const std::string kShouldPredictObjectVelocitiesKey("should_predict_object_velocities");
const std::string kVelocityPredictionLookaheadTimeKey("velocity_prediction_lookahead_time_s");
const std::string kObjectReactionTimeKey("agent_reaction_time_s");


robot_simulator_params_t::robot_simulator_params_t(const utils::ConfigFile& controllerConfig, const utils::ConfigFile& robotConfig)
: kinematicControlLawParams(controllerConfig)
, joystickControlLawParams(controllerConfig)
, robotPlantModelParams(robotConfig)
{
    // simulator-specific parameter modifications here
    kinematicControlLawParams.maxLinearVelocity  += 0.1; // you want to add some margin to the limits in the robot simulator as the optimizer will often test values outside the search bound.
    kinematicControlLawParams.maxAngularVelocity += 0.1;

    joystickControlLawParams.useAdaptiveParams        = false; // don't generally need this special heuristic in the simulator
    joystickControlLawParams.shouldOutputDebugMessage = false; // don't need the debug outputs in the simulator
}


dynamic_object_filter_params_t::dynamic_object_filter_params_t(const utils::ConfigFile& config)
: staleObjectTimeUs(config.getValueAsInt32(kDynamicObjectFilterHeading, kStaleTimeKey) * 1000LL)
, maxObjectSpeed(config.getValueAsFloat(kDynamicObjectFilterHeading, kMaxObjectSpeedKey))
, maxTrustedVelocityStd(config.getValueAsDouble(kDynamicObjectFilterHeading, kMaxVelocityStdKey))
, startUntrustedVelocityStd(config.getValueAsDouble(kDynamicObjectFilterHeading, kStartUntrustedVelocityStdKey))
, minGoalProbability(config.getValueAsDouble(kDynamicObjectFilterHeading, kMinGoalProbabilityKey))
, shouldSlowdownObjectsBehindRobot(config.getValueAsBool(kDynamicObjectFilterHeading, kShouldSlowdownObjectBehindRobotKey))
, slowdownObjectConeAngle(config.getValueAsFloat(kDynamicObjectFilterHeading, kSlowdownConeAngleKey) * M_PI / 180.0)
, ignoreObjectConeRadius(config.getValueAsFloat(kDynamicObjectFilterHeading, kBlindConeRadiusKey))
{
    assert(staleObjectTimeUs > 0);
    assert(maxObjectSpeed > 1.0f);
    assert(slowdownObjectConeAngle >= 0.0f);
    assert(slowdownObjectConeAngle <  1.6f);
    assert(ignoreObjectConeRadius  >= 0.0f);
    assert(maxTrustedVelocityStd > startUntrustedVelocityStd);
    assert(minGoalProbability > 0.0f);
}


dynamic_object_simulator_params_t::dynamic_object_simulator_params_t(const utils::ConfigFile& config)
: shouldPredictObjectVelocities(config.getValueAsBool(kDynamicObjectSimlatorHeading, kShouldPredictObjectVelocitiesKey))
, lookaheadTime(config.getValueAsFloat(kDynamicObjectSimlatorHeading, kVelocityPredictionLookaheadTimeKey))
, reactionTime(config.getValueAsFloat(kDynamicObjectSimlatorHeading, kObjectReactionTimeKey))
{
    assert(lookaheadTime >  0.5f);
    assert(reactionTime >= 0.0f);
}

} // namespace mpepc
} // namespace vulcan
