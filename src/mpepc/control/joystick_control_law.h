/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file   joystick_control_law.h
 * \author Jong Jin Park
 *
 * Declaration of JoystickControlLaw.
 */

#ifndef MPEPC_JOYSTICK_CONTROL_LAW_H
#define MPEPC_JOYSTICK_CONTROL_LAW_H

#include "core/motion_state.h"
#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/control/params.h"
#include "robot/model/params.h"

namespace vulcan
{
namespace robot
{
struct joystick_command_t;
}

namespace mpepc
{

/**
 * JoystickControlLaw is a model-based velocity tracking controller for Vulcan, with both the feedforward and the
 * feedback components. It uses a steady-state mapping from joystick position to velocity of the Curtis controller
 * as a feedforward model. For feedback, small (bounded) additive proportional control is added to the feedforward
 * signal based on the difference between the provided reference velocity and the current measured speed.
 *
 * This can be understood as a sophisticated version of a PI-control where the I-component is replaced with the
 * feedforward. NOTE: May want to add D term to this. NOTE: In the current implementation, the feedforward model is hard
 * coded with fixed parameters, but in the not too distant future this should be replaced with a learned model.
 */
class JoystickControlLaw
{
public:
    /**
     * Constructor for JoystickControlLaw.
     */
    JoystickControlLaw(const joystick_control_law_params_t& params,
                       const robot::differential_motors_plant_params_t& robotParams);

    /**
     * computeOuput uses the current velocity and the reference velocity to compute the joystick command to be sent to
     * the robot, via feedforward model and feedback compensation. \param    currentVelocity         Current velocity
     * measurement. \param    reference               Control_law_output_t structure containing reference information
     * (target velocity, heading and curvature). \return   Joystick positions (commands to the robot) to be sent out.
     */
    robot::joystick_command_t computeOutput(const motion_state_t& robotState, const control_law_output_t& reference);

private:
    /**
     * computeFeedforwardCommand computes the base joystick command output as a function of the reference input using an
     * internal model and conditional I-control. \param    reference               Control_law_output_t structure
     * containing reference information (target velocity, heading and curvature). \return   Joystick positions to be
     * commanded
     */
    robot::joystick_command_t computeFeedforwardCommand(const control_law_output_t& reference);
    // NOTE: Feedforward model is responsible for guranteeing convergence to the reference.

    /**
     * computeFeedbackCommand computes bounded control effort to compensate for the current velocity error observed via
     * PD-control. \param    currentState            Current estimate of the robot state. \param    reference
     * Control_law_output_t structure containing reference information (target velocity, heading and curvature).
     */
    robot::joystick_command_t computeFeedbackCommand(const motion_state_t& currentState,
                                                     const control_law_output_t& reference);
    // NOTE: For velocity control feedback without memory cannot gurantee the steady-state convergence, but can reduce
    // error and improve transient behaviors.

    /**
     * updateStatus updates the operational status of the controller, which are isTryingToMove_ and isNotMoving_.
     * \param    currentState            Current estimate of the robot state.
     * \param    reference               Control_law_output_t structure containing reference information (target
     * velocity, heading and curvature).
     */
    void updateStatus(const motion_state_t& currentState, const control_law_output_t& reference);

    // clamping joystick position so it is within an allowed range
    int16_t clampJoystickPosition(int16_t position);

    // other storage variables
    motion_state_t previousState_;
    control_law_output_t previousReference_;

    // status indicators for conditional I-control
    bool isTryingToMove_;
    bool isNotMoving_;
    bool shouldTryHarderToMove_;
    float integralError_;

    // parameters
    joystick_control_law_params_t params_;
    robot::differential_motors_plant_params_t robotParams_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_JOYSTICK_CONTROL_LAW_H