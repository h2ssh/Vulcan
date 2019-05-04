/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     joystick_control_law.cpp
* \author   Jong Jin Park
*
* Definition of JoystickControlLaw.
*/

#include <mpepc/control/joystick_control_law.h>
#include <robot/commands.h>
#include <utils/timestamp.h>
#include <cmath>
#include <iostream>

namespace vulcan
{

namespace mpepc
{

// joystick constants
const int16_t MAX_JOYSTICK =  100;
const int16_t MIN_JOYSTICK = -100;


JoystickControlLaw::JoystickControlLaw(const joystick_control_law_params_t& params, const robot::differential_motors_plant_params_t& robotParams)
: isTryingToMove_(false)
, isNotMoving_(false)
, shouldTryHarderToMove_(false)
, params_(params)
, robotParams_(robotParams)
{
}

robot::joystick_command_t JoystickControlLaw::computeOutput(const motion_state_t& robotState, const control_law_output_t& reference)
{
    // update status
    updateStatus(robotState, reference);
    
    // compute model-based feedforward command, which is a steady-state mapping between joystick position and velocities
    robot::joystick_command_t feedforwardCommand = computeFeedforwardCommand(reference);
    
    // add feedback compensation to the feedforward command with P-control
    robot::joystick_command_t feedbackCompensation = computeFeedbackCommand(robotState, reference);
    
    // cmopute final output and clamp joystick positions
    robot::joystick_command_t output;
    output.forward = clampJoystickPosition(feedforwardCommand.forward + feedbackCompensation.forward);
    output.left    = clampJoystickPosition(feedforwardCommand.left    + feedbackCompensation.left);
    
    // store info for next update
    previousState_     = robotState;
    previousReference_ = reference;

    if(params_.shouldOutputDebugMessage)
    {
        std::cout<<"DEBUG JOYSTICK CONTROL: reference velocity : ("<<reference.linearVelocity<<','<<reference.angularVelocity<<")\n";
        std::cout<<"DEBUG JOYSTICK CONTROL: current velocity   : ("<<robotState.velocity.linear<<','<<robotState.velocity.angular<<")\n"; 
        std::cout<<"DEBUG JOYSTICK CONTROL: controller status  : ("<<isTryingToMove_<<','<<isNotMoving_<<','<<integralError_<<','<<shouldTryHarderToMove_<<")\n";
        std::cout<<"DEBUG JOYSTICK CONTROL: Feedforward command: ("<<feedforwardCommand.forward<<','<<feedforwardCommand.left<<")\n"; 
        std::cout<<"DEBUG JOYSTICK CONTROL: Feedback command   : ("<<feedbackCompensation.forward<<','<<feedbackCompensation.left<<")\n"; 
        std::cout<<"DEBUG JOYSTICK CONTROL: Combined output    : ("<<output.forward<<','<<output.left<<")\n"; 
    }
    
    return output;
}


robot::joystick_command_t JoystickControlLaw::computeFeedbackCommand(const motion_state_t& currentState, const control_law_output_t& reference)
{
    robot::joystick_command_t output;
    
    // velocities for P- control
    velocity_t currentVelocity;
    if(currentState.haveWheels)
    {
        currentVelocity.linear  = (currentState.differentialWheels.rightWheel.speed + currentState.differentialWheels.leftWheel.speed) / 2.0f;
        currentVelocity.angular = (currentState.differentialWheels.rightWheel.speed - currentState.differentialWheels.leftWheel.speed) / robotParams_.wheelbase; 
    }
    else
    {
        currentVelocity = currentState.velocity;
    }
    
    // reference accelerations for D- control
    acceleration_t referenceAcceleration;
    if(currentState.timestamp == 0 || currentState.timestamp < previousState_.timestamp)
    {
        referenceAcceleration.linear  = 0;
        referenceAcceleration.angular = 0;
    }
    else
    {
        float timeDelta = utils::usec_to_sec(currentState.timestamp - previousState_.timestamp);
        referenceAcceleration.linear  = (reference.linearVelocity  - previousReference_.linearVelocity)  / timeDelta;
        referenceAcceleration.angular = (reference.angularVelocity - previousReference_.angularVelocity) / timeDelta;
    }
    
    
    // parameters
    float linearVelocityPGain  = params_.linearVelocityPGain;
    float linearVelocityDGain  = params_.linearVelocityDGain;
    float angularVelocityPGain = params_.angularVelocityPGain;
    float angularVelocityDGain = params_.angularVelocityDGain;
    float maxControlEffort     = params_.maxControlEffort;
    
    if(params_.useAdaptiveParams)
    {
        float kGainMultiplier   = 3.0;
        float kGainIntercept    = 0.3;
        float kEffortMultiplier = 1.0;
        float kEffortIntercept  = 0.5;
        
        // vary gain as a function of target velocities. Slower target speed allows higher gain.
        linearVelocityPGain   = fmax(1.0 + (kGainMultiplier - 1.0)/kGainIntercept*(kGainIntercept - fabs(reference.linearVelocity) ), 1.0)  * linearVelocityPGain;
        angularVelocityPGain  = fmax(1.0 + (kGainMultiplier - 1.0)/kGainIntercept*(kGainIntercept - fabs(reference.angularVelocity) ), 1.0) * angularVelocityPGain;
        
        // vary maximum control effort based on current speed. Low speed (closer to being stationary) means higher max control effort.
        float currentSpeed     = sqrt(currentVelocity.linear*currentVelocity.linear + currentVelocity.angular*currentVelocity.angular);
        maxControlEffort       = fmax(1.0 + (kEffortMultiplier - 1.0)/kEffortIntercept*(kEffortIntercept - currentSpeed), 1.0) * maxControlEffort;
    }
    
    
    // actual computation
    float joystickForwardP = linearVelocityPGain*(reference.linearVelocity - currentVelocity.linear);
    float joystickForwardD = linearVelocityDGain*(referenceAcceleration.linear - currentState.acceleration.linear);
    float joystickForwardCompensation = fmin(fabs(joystickForwardP + joystickForwardD), maxControlEffort);
    joystickForwardCompensation       = copysign(joystickForwardCompensation, joystickForwardP + joystickForwardD);
    
    float joystickLeftP = angularVelocityPGain*(reference.angularVelocity - currentVelocity.angular);
    float joystickLeftD = angularVelocityDGain*(referenceAcceleration.angular - currentState.acceleration.angular);
    float joystickLeftCompensation = fmin(fabs(joystickLeftP + joystickLeftD), maxControlEffort);
    joystickLeftCompensation       = copysign(joystickLeftCompensation, joystickLeftP + joystickLeftD);
    
    output.forward = joystickForwardCompensation;
    output.left    = joystickLeftCompensation;
    
    return output;
}


robot::joystick_command_t JoystickControlLaw::computeFeedforwardCommand(const control_law_output_t& reference)
{
    robot::joystick_command_t output;
    
    // TODO: ideally, have steady_state_joystick_positions = inverse_model(reference_velocity) and have the inverse model live within the planner.
    // i.e. if(useLearedModel) { ouput = inverseModel->steadyStateCommand(const velocity_t& referenceVelocity) }

    float   rightMotorCommand;
    float   leftMotorCommand;
    int16_t joystickForward;
    int16_t joystickLeft;
    
    // reference velocities
    float referenceLinearVelocity  = reference.linearVelocity;
    float referenceAngularVelocity = reference.angularVelocity;
    
    // conditional I- control, implemented via artificial amplification of error signal.
    // This preserves the ratio between the linear and the angular velocity, which is important for maintaining curvature.
    if(shouldTryHarderToMove_)
    {
        float multiplier;
        multiplier = fmin(fabs(integralError_), 3.0); // multiplier with max = 3.0
        multiplier = fmax(multiplier, 2.0); // multiplier min = 2.0
        
        referenceLinearVelocity  *= multiplier;
        referenceAngularVelocity *= multiplier;
    }
    
    // conveting vehicle velocity to wheel speeds
    float desiredRightWheelSpeed = referenceLinearVelocity + (robotParams_.wheelbase/2.0)*referenceAngularVelocity;
    float desiredLeftWheelSpeed  = referenceLinearVelocity - (robotParams_.wheelbase/2.0)*referenceAngularVelocity;
    
    // wheel speeds to motor commands
    if(fabs(desiredRightWheelSpeed) < 0.005) // if desired speed steady state is very small
    {
        rightMotorCommand = 0;
    }
    else
    {
        rightMotorCommand = robotParams_.motorBeta/robotParams_.motorAlpha*desiredRightWheelSpeed + copysign(robotParams_.motorMu*robotParams_.motorGamma/robotParams_.motorAlpha, desiredRightWheelSpeed);
//         rightMotorCommand = std::copysign(fabs(desiredRightWheelSpeed)*40.0 + 7.2, desiredRightWheelSpeed);
    }
    
    if(fabs(desiredLeftWheelSpeed) < 0.005) // if desired speed steady state is very small
    {
        leftMotorCommand = 0;
    }
    else
    {
        leftMotorCommand = robotParams_.motorBeta/robotParams_.motorAlpha*desiredLeftWheelSpeed + copysign(robotParams_.motorMu*robotParams_.motorGamma/robotParams_.motorAlpha, desiredLeftWheelSpeed);
//        leftMotorCommand = std::copysign(fabs(desiredLeftWheelSpeed)*40.0 + 7.2, desiredLeftWheelSpeed);
    }
    
    // motor commands to joystick commands
    joystickForward = static_cast<int16_t>((rightMotorCommand + leftMotorCommand)/2.0);
    
    float turnRate = (joystickForward == 0) ? robotParams_.turnRateInPlace : robotParams_.turnRateBase*(1 - robotParams_.turnReductionRate*(fabs(static_cast<float>(joystickForward))));
    joystickLeft   = static_cast<int16_t>(rightMotorCommand - leftMotorCommand)/2.0/turnRate;
    
    // modificiation of joystick axis if required    
    if(params_.shouldModifyJoystickAxis)
    {
        int16_t modifiedJoystickForward = -sin(params_.axisRotation)*joystickLeft + cos(params_.axisRotation)*joystickForward;
        int16_t modifiedJoystickLeft    =  cos(params_.axisRotation)*joystickLeft + sin(params_.axisRotation)*joystickForward;
        
        if(modifiedJoystickForward != 0)
        {
            modifiedJoystickLeft += params_.leftBias; // add fixed-amount left bias. Negative value means right bias.
        }
        
        joystickForward = clampJoystickPosition(modifiedJoystickForward);
        joystickLeft    = clampJoystickPosition(modifiedJoystickLeft);
    }
    else
    {
        joystickForward = clampJoystickPosition(joystickForward);
        joystickLeft    = clampJoystickPosition(joystickLeft);
    }
    
    output.forward = joystickForward;
    output.left    = joystickLeft;

    return output;
}


void JoystickControlLaw::updateStatus(const motion_state_t& currentState, const control_law_output_t& reference)
{
    isTryingToMove_ = fabs(reference.linearVelocity)     > 0.00 || fabs(reference.angularVelocity)     > 0.00;
    isNotMoving_    = fabs(currentState.velocity.linear) < 0.05 && fabs(currentState.velocity.angular) < 0.05;
    
    bool isMovingFastEnough = fabs(currentState.velocity.linear) > 0.2 || fabs(currentState.velocity.angular) > 0.2;
    
    if(isTryingToMove_) // update variables for integral control
    {
        integralError_ += fabs(reference.angularVelocity - currentState.velocity.angular); // just using angular component for error.
        shouldTryHarderToMove_ = isNotMoving_ && integralError_ > 1.0; // if is trying to move and is not moving for some time then it should try harder!
    }
    
    if(!isTryingToMove_ || isMovingFastEnough) // conditions for resetting integral control
    {
        if(params_.shouldOutputDebugMessage)
        {
            std::cout<<"DEBUG: Reset condition activated\n";
        }
        integralError_         = 0.0;
        shouldTryHarderToMove_ = false;
    }
    
    if(params_.shouldOutputDebugMessage)
    {
        std::cout<<"DEBUG: Integral error: "<<integralError_<<'\n';
        std::cout<<"DEBUG: isMovingFastEnough: "<<isMovingFastEnough<<'\n';
    }
}


int16_t JoystickControlLaw::clampJoystickPosition(int16_t position)
{
    if(position > MAX_JOYSTICK)
    {
        position = MAX_JOYSTICK;
//         std::cout << "WARNING!: SATURATED CONTROL EFFORT!" << '\n';
    }

    if(position < MIN_JOYSTICK)
    {
        position = MIN_JOYSTICK;
//         std::cout << "WARNING!: SATURATED CONTROL EFFORT!" << '\n';
    }

    return position;
}


} // namespace planner
} // namespace vulcan
