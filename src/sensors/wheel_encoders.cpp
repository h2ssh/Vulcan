/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     wheel_encoders.cpp
 * \author   Collin Johnson
 *
 * Definition of WheelEncoders and create_wheel_encoders factory.
 */

#include "sensors/wheel_encoders.h"
#include "core/angle_functions.h"
#include "sensors/phidget_encoder_board.h"
#include "sensors/remote_encoders.h"
#include "system/module_communicator.h"
#include "utils/timestamp.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

#define DEBUG_ODOMETRY

namespace vulcan
{
namespace sensors
{

std::unique_ptr<WheelEncoders> create_wheel_encoders(const std::string& type, const wheel_encoders_params_t& params)
{
    std::cout << "Creating Wheel Encoders" << std::endl;
    if (type == REMOTE_ENCODERS_TYPE) {
        std::cout << "REMOTE_ENCODERS" << std::endl;
        return std::unique_ptr<WheelEncoders>(new RemoteEncoders(params));
    }
#ifndef VGO_ROBOT
    else if (type == PHIDGET_ENCODER_BOARD_TYPE) {
        std::cout << "PHIDGET_ENCODERS" << std::endl;
        return std::unique_ptr<WheelEncoders>(new PhidgetEncoderBoard(params));
    }
#endif
    else {
        std::cerr << "ERROR:create_wheel_encoders: Unknown encoder type: " << type << std::endl;

        exit(-1);
    }
    std::cout << "Normal encoders" << std::endl;
    return std::unique_ptr<WheelEncoders>();
}


WheelEncoders::WheelEncoders(const wheel_encoders_params_t& params)
: lastOutputTime(0)
, initialized(false)
, params(params)
{
    cumulative.id = 0;
}


void WheelEncoders::initialize(system::ModuleCommunicator& communicator)
{
    // Nothing to do for the normal WheelEncoders
}


odometry_t WheelEncoders::update(void)
{
    currentEncoders = getEncoders();

    if (!initialized) {
        previousEncoders = currentEncoders;
        initialized = true;
    }

    calculateWheelMotion();
    calculateDeadReckoningPose();

    ++cumulative.id;
    cumulative.timestamp = currentEncoders.timestamp;

    previousEncoders = currentEncoders;

#ifdef DEBUG_ODOMETRY
    if (utils::system_time_us() - lastOutputTime > 250000) {
        std::cout << "DEBUG:WheelEncoders: (x,y,theta):(" << cumulative.x << ',' << cumulative.y << ','
                  << cumulative.theta << ")\n"
                  << "                 (dLeft, dRight):(" << currentEncoders.deltaLeftWheel << ','
                  << currentEncoders.deltaRightWheel << ")\n"
                  << "         (leftIndex, rightIndex):(" << currentEncoders.leftIndexPulseTotal << ','
                  << currentEncoders.rightIndexPulseTotal << ")\n"
                  << "             (leftRPM, rightRPM):(" << currentEncoders.leftRPM << ',' << currentEncoders.rightRPM
                  << ")\n";
    }
#endif

    return cumulative;
}


void WheelEncoders::send(system::ModuleCommunicator& transmitter)
{
    transmitter.sendMessage(cumulative);
    transmitter.sendMessage(currentEncoders);
}


void WheelEncoders::calculateWheelMotion(void)
{
    // Update based on the difference between the previous read and currently read ticks because the wheel encoders
    // update process could be slower than the encoders update, thus the wheel deltas could be wrong
    currentEncoders.deltaLeftWheel = (currentEncoders.leftTicksTotal - previousEncoders.leftTicksTotal)
      * currentEncoders.leftWheelCircumference / currentEncoders.leftTicksPerRevolution;
    currentEncoders.deltaRightWheel = (currentEncoders.rightTicksTotal - previousEncoders.rightTicksTotal)
      * currentEncoders.rightWheelCircumference / currentEncoders.rightTicksPerRevolution;
}


void WheelEncoders::calculateDeadReckoningPose(void)
{
    const double distance = (currentEncoders.deltaLeftWheel + currentEncoders.deltaRightWheel) * 0.5;
    const double rotation = (currentEncoders.deltaRightWheel - currentEncoders.deltaLeftWheel) / currentEncoders.wheelbase;

    cumulative.x += distance * std::cos(cumulative.theta + rotation / 2);
    cumulative.y += distance * std::sin(cumulative.theta + rotation / 2);
    cumulative.theta = angle_sum(cumulative.theta, rotation);

    cumulative.translation = distance;
    cumulative.rotation = rotation;
}

}   // namespace sensors
}   // namespace vulcan
