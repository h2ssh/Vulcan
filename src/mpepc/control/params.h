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
 * Declaration of params structs for running the KinematicControlLaw and JoystickConrolLaw.
 */

#ifndef MPEPC_CONTROL_LAW_PARAMS_H
#define MPEPC_CONTROL_LAW_PARAMS_H

#include <cstdint>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace mpepc
{

/**
 * kinematic_control_law_params_t defines the parameters to use for determining velocities with the kinematic
 * control law, published in Park and Kuipers, ICRA-11
 */
struct kinematic_control_law_params_t
{
    // parameters for path shape and speed profile
    float k1;   // k1 and k2 determine path shape
    float k2;
    float beta;   // beta and lambda determine velocity profile
    float lambda;

    // Parameters determining behavior of the robot near the target
    float slowdownRadius;
    float convergenceRadius;
    float convergenceAngle;
    int64_t convergenceTimeUs;

    // Parameters for controlling the angular velocity when turning in place at the target
    float angularVelocityTurnInPlace;

    // Extra parameters for limiting velocities for extra safety if desired.
    // Graceful motion control law itself provides bounds on velocity, acceleration and jerk by construction, via
    // velocityGain.
    float maxLinearVelocity;
    float maxAngularVelocity;

    kinematic_control_law_params_t(void){};
    kinematic_control_law_params_t(const utils::ConfigFile& config);
};


struct joystick_control_law_params_t
{
    bool shouldModifyJoystickAxis;
    float axisRotation;   // 0.0
    float leftBias;

    bool useAdaptiveParams;   // will vary base parameters based on the current speed and the target velocities
    float maxControlEffort;
    float linearVelocityPGain;
    float linearVelocityDGain;
    float angularVelocityPGain;
    float angularVelocityDGain;

    // This controls debug message outputs to terminal.
    bool shouldOutputDebugMessage;

    joystick_control_law_params_t(void){};
    joystick_control_law_params_t(const utils::ConfigFile& config);
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_CONTROL_LAW_PARAMS_H
