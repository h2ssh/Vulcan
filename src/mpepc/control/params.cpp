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
 * Definition of parsers for the various parameters structs for the controllers.
 */

#include "mpepc/control/params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"
#include <cassert>

namespace vulcan
{

namespace mpepc
{

const std::string kKinematicControlLawHeading("KinematicControlLawParameters");
const std::string kK1Key("k1");
const std::string kK2Key("k2");
const std::string kBetaKey("beta");
const std::string kLambdaKey("lambda");
const std::string kSlowdownRadiusKey("slowdown_radius");
const std::string KConvergeRadiusKey("convergence_radius");
const std::string KConvergeAngleKey("convergence_angle");
const std::string kConvergeTimeKey("convergence_time_ms");
const std::string kTurnInPlaceVelKey("turn_in_place_velocity");
const std::string kMaxLinVelKey("maximum_linear_velocity");
const std::string kMaxAngVelKey("maximum_angular_velocity");

const std::string kJoystickControlLawHeading("JoystickControlLawParameters");
const std::string kModifyJoystickAxisKey("should_modify_joystick_axis");
const std::string kAxisRotationKey("axis_rotation");
const std::string kLeftBiasKey("left_bias");
const std::string kUseAdaptiveParamsKey("use_adaptive_params");
const std::string kMaxControlEffortKey("maximum_control_effort");
const std::string kLinearVelocityPGainKey("linear_velocity_p_gain");
const std::string kLinearVelocityDGainKey("linear_velocity_d_gain");
const std::string kAngularVelocityPGainKey("angular_velocity_p_gain");
const std::string kAngularVelocityDGainKey("angular_velocity_d_gain");
const std::string kDebugOutputKey("debug_outputs");


kinematic_control_law_params_t::kinematic_control_law_params_t(const utils::ConfigFile& config)
: k1(config.getValueAsFloat(kKinematicControlLawHeading, kK1Key))
, k2(config.getValueAsFloat(kKinematicControlLawHeading, kK2Key))
, beta(config.getValueAsFloat(kKinematicControlLawHeading, kBetaKey))
, lambda(config.getValueAsFloat(kKinematicControlLawHeading, kLambdaKey))
, slowdownRadius(config.getValueAsFloat(kKinematicControlLawHeading, kSlowdownRadiusKey))
, convergenceRadius(config.getValueAsFloat(kKinematicControlLawHeading, KConvergeRadiusKey))
, convergenceAngle(config.getValueAsFloat(kKinematicControlLawHeading, KConvergeAngleKey))
, convergenceTimeUs(config.getValueAsInt32(kKinematicControlLawHeading, kConvergeTimeKey) * 1000ll)
, angularVelocityTurnInPlace(config.getValueAsFloat(kKinematicControlLawHeading, kTurnInPlaceVelKey))
, maxLinearVelocity(config.getValueAsFloat(kKinematicControlLawHeading, kMaxLinVelKey))
, maxAngularVelocity(config.getValueAsFloat(kKinematicControlLawHeading, kMaxAngVelKey))
{
    assert(k1 > 0.0f);
    assert(k2 > 0.0f);
    assert(beta > 0.0f);
    assert(lambda > 0.0f);
}


joystick_control_law_params_t::joystick_control_law_params_t(const utils::ConfigFile& config)
: shouldModifyJoystickAxis(config.getValueAsBool(kJoystickControlLawHeading, kModifyJoystickAxisKey))
, axisRotation(config.getValueAsFloat(kJoystickControlLawHeading, kAxisRotationKey))
, leftBias(config.getValueAsFloat(kJoystickControlLawHeading, kLeftBiasKey))
, useAdaptiveParams(config.getValueAsBool(kJoystickControlLawHeading, kUseAdaptiveParamsKey))
, maxControlEffort(config.getValueAsFloat(kJoystickControlLawHeading, kMaxControlEffortKey))
, linearVelocityPGain(config.getValueAsFloat(kJoystickControlLawHeading, kLinearVelocityPGainKey))
, linearVelocityDGain(config.getValueAsFloat(kJoystickControlLawHeading, kLinearVelocityDGainKey))
, angularVelocityPGain(config.getValueAsFloat(kJoystickControlLawHeading, kAngularVelocityPGainKey))
, angularVelocityDGain(config.getValueAsFloat(kJoystickControlLawHeading, kAngularVelocityDGainKey))
, shouldOutputDebugMessage(config.getValueAsBool(kJoystickControlLawHeading, kDebugOutputKey))
{
    assert(maxControlEffort > -1.0f);
    assert(maxControlEffort < 100.0f);
    assert(linearVelocityPGain >= 0.0f);
    assert(linearVelocityDGain >= 0.0f);
    assert(angularVelocityPGain >= 0.0f);
    assert(angularVelocityDGain >= 0.0f);
}

}   // namespace mpepc
}   // namespace vulcan
