/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     bridge.cpp
* \author   Collin Johnson
*
* Definition of VGoBridge.
*/

#include "robot/vgo_bridge/bridge.h"
#include "robot/vgo_bridge/service.h"
#include "robot/commands.h"
#include "core/odometry.h"
#include "utils/timestamp.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

namespace vulcan
{
namespace robot
{

VGoBridge::VGoBridge(void)
    : haveInitializedTicks(false)
    , lastLeftTicks(0)
    , lastRightTicks(0)
    , encoderMessageId(0)
{
    const std::string VELOCITY_CHANNEL("/vgo/cmd_vel");
    const std::string ENCODER_CHANNEL ("encoder_data");
    const std::string BUMPER_CHANNEL  ("bumper_data");
    const std::string POWER_CHANNEL   ("power_data");
    const std::string IR_CHANNEL      ("ir_data");

    const uint32_t QUEUE_SIZE = 100;

    encoderSubscriber = rosHandle.subscribe(ENCODER_CHANNEL, QUEUE_SIZE, &VGoBridge::handleVGoEncoders, this);
    bumperSubscriber  = rosHandle.subscribe(BUMPER_CHANNEL,  QUEUE_SIZE, &VGoBridge::handleVGoBumpers,  this);
    irSubscriber      = rosHandle.subscribe(IR_CHANNEL,      QUEUE_SIZE, &VGoBridge::handleVGoIR,       this);
    powerSubscriber   = rosHandle.subscribe(POWER_CHANNEL,   QUEUE_SIZE, &VGoBridge::handleVGoPower,    this);

    commandPublisher = rosHandle.advertise<geometry_msgs::Twist>(VELOCITY_CHANNEL, QUEUE_SIZE);

    communicator.subscribeTo<commanded_velocity_t>(this);
//     receiver.subscribeToMessage<vgo_service_t>(this);

    receiverThread.attachTask(&communicator);
    receiverThread.start();
}


void VGoBridge::handleData(const commanded_velocity_t& command, const std::string& channel)
{
    geometry_msgs::Twist velocity;

    velocity.linear.x = command.linearVelocity;
    velocity.linear.y = command.angularVelocity;

    commandPublisher.publish(velocity);
}


void VGoBridge::handleData(const vgo_service_t& service, const std::string& channel)
{
    processServiceRequest(service);
}


void VGoBridge::processServiceRequest(const vgo_service_t& service)
{

}


void VGoBridge::handleVGoEncoders(const vgo_msgs::Encoders::ConstPtr& encoderData)
{
    const float WHEEL_CIRCUMFERENCE  = 0.177795f;
    const float TICKS_PER_REVOLUTION = 1865.0f;

    encoder_data_t encoders;

    if(!haveInitializedTicks)
    {
        haveInitializedTicks = true;
        lastLeftTicks        = encoderData->left;
        lastRightTicks       = encoderData->right;
    }

    encoders.timestamp       = utils::system_time_us();
    encoders.id              = encoderMessageId++;
    encoders.leftTicksTotal  = encoderData->left;
    encoders.rightTicksTotal = encoderData->right;
    encoders.deltaLeftWheel  = (encoderData->left - lastLeftTicks)   / TICKS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;
    encoders.deltaRightWheel = (encoderData->right - lastRightTicks) / TICKS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;

    lastLeftTicks  = encoderData->left;
    lastRightTicks = encoderData->right;

    encoders.leftTicksPerRevolution = encoders.rightTicksPerRevolution = TICKS_PER_REVOLUTION;
    encoders.leftWheelCircumference = encoders.rightWheelCircumference = WHEEL_CIRCUMFERENCE;
    encoders.wheelbase              = 0.3048;

    std::cout<<"Ticks(l,r): ("<<lastLeftTicks<<','<<lastRightTicks<<")\n";

    communicator.sendMessage(encoders);
}


void VGoBridge::handleVGoBumpers(const vgo_msgs::Bumper::ConstPtr& bumperData)
{
    std::cout<<"Bump:(l,r)"<<bumperData->left_bumper<<','<<bumperData->right_bumper<<'\n';
}


void VGoBridge::handleVGoIR(const vgo_msgs::ir_msg::ConstPtr& irData)
{

}


void VGoBridge::handleVGoPower(const vgo_msgs::PowerButton::ConstPtr& powerData)
{

}

} // namespace robot
} // namespace vulcan
