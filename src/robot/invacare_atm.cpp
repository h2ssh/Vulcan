/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     invacare_atm.cpp
 * \author   Phil McKenna, Collin Johnson
 * Further modifications by R Gaynier 12/04/12, all changes noted with comments and comments added where they help me
 * understand the differences between the Invacare and Quantum 6000 implementations.
 *
 * Definition of InvacareATM.
 */

#include "robot/invacare_atm.h"
#include "robot/commands.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"   // Added to match quantum_6000.cpp, may not be needed.
#include <iomanip>             // Added to match quantum_6000.cpp, may not be needed.
#include <iostream>
#include <sys/select.h>

namespace vulcan
{
namespace robot
{

const int MIN_JOYSTICK_FORWARD = 5;
const int MIN_JOYSTICK_LEFT = 5;
const char JOYSTICK_GAIN = 64;

InvacareATM::InvacareATM(const std::string& joystickPort, const std::string& controllerPort)
: joystick(joystickPort, utils::BAUD_38400, utils::EIGHT_N_2, true, 15)       // equivalent to joystickBus
, controller(controllerPort, utils::BAUD_38400, utils::EIGHT_N_2, true, 15)   // equivalent to controllerBus
, gotResponse(3)
, controlEnabled(false)
{
    // setting gotResponse to 3 allows dropped responses from controller just in case of noise during
    // flipping of the toggle switch

    // Initialize control message to joystick idle position with max gain
    controlMessage[0] = 96;
    controlMessage[1] = 192;
    controlMessage[2] = 191;
    controlMessage[3] = 255;
    controlMessage[4] = 206;
    controlMessage[5] = 128;
    controlMessage[6] = 132;
    controlMessage[7] = 128;
    controlMessage[8] = 207;
    controlMessage[9] = 15;

    // do not change this remains constant
    responseToJoystick[0] = 33;
    responseToJoystick[1] = 144;
    responseToJoystick[2] = 128;
    responseToJoystick[3] = 128;
    responseToJoystick[4] = 206;
    responseToJoystick[5] = 15;

    // do not change this remains constant
    handshakeResponse[0] = 5;
    handshakeResponse[1] = 128;
    handshakeResponse[2] = 250;
    handshakeResponse[3] = 15;
}

InvacareATM::InvacareATM(const wheelchair_joystick_calibration_t& calibration,
                         const std::string& joystickPort,
                         const std::string& controllerPort)
: joystick(joystickPort, utils::BAUD_38400, utils::EIGHT_N_2, true, 15)       // equivalent to joystickBus
, controller(controllerPort, utils::BAUD_38400, utils::EIGHT_N_2, true, 15)   // equivalent to controllerBus
, calibration(calibration)
, gotResponse(3)
, controlEnabled(false)
{
    // setting gotResponse to 3 allows dropped responses from controller just in case of noise during
    // flipping of the toggle switch

    // Initialize control message to joystick idle position with max gain
    controlMessage[0] = 96;
    controlMessage[1] = 192;
    controlMessage[2] = 191;
    controlMessage[3] = 255;
    controlMessage[4] = 206;
    controlMessage[5] = 128;
    controlMessage[6] = 132;
    controlMessage[7] = 128;
    controlMessage[8] = 207;
    controlMessage[9] = 15;

    // do not change this remains constant
    responseToJoystick[0] = 33;
    responseToJoystick[1] = 144;
    responseToJoystick[2] = 128;
    responseToJoystick[3] = 128;
    responseToJoystick[4] = 206;
    responseToJoystick[5] = 15;

    // do not change this remains constant
    handshakeResponse[0] = 5;
    handshakeResponse[1] = 128;
    handshakeResponse[2] = 250;
    handshakeResponse[3] = 15;
}

InvacareATM::~InvacareATM(void)
{
    // Nothing to do for now
}


robot::commanded_velocity_t
  InvacareATM::convertJoystickCommandToCommandedVelocity(const robot::joystick_command_t& joystick) const
{
    robot::commanded_velocity_t command;

    float angularSlope = 0.0f;

    if (joystick.forward == 0) {
        angularSlope = (joystick.left < 0) ? calibration.turnInPlaceNegativeAngularVelocityRatio
                                           : calibration.turnInPlacePositiveAngularVelocityRatio;
    } else {
        angularSlope = (joystick.left < 0) ? calibration.translatingNegativeAngularVelocityRatio
                                           : calibration.translatingPositiveAngularVelocityRatio;
    }

    if (joystick.forward >= MIN_JOYSTICK_FORWARD) {
        command.linearVelocity = (joystick.forward - MIN_JOYSTICK_FORWARD) * calibration.positiveLinearVelocityRatio;
    } else if (joystick.forward <= -MIN_JOYSTICK_FORWARD) {
        command.linearVelocity = (joystick.forward + MIN_JOYSTICK_FORWARD) * calibration.negativeLinearVelocityRatio;
    }

    command.timestamp = joystick.timestamp;
    command.angularVelocity = joystick.left * angularSlope;

    // If in the joystick deadband, not actually moving
    if (abs(joystick.forward) < MIN_JOYSTICK_FORWARD && abs(joystick.left) < MIN_JOYSTICK_LEFT) {
        command.linearVelocity = 0.0f;
        command.angularVelocity = 0.0f;
    }

    return command;
}


robot::joystick_command_t
  InvacareATM::convertVelocityCommandToJoystickCommand(const robot::velocity_command_t& command) const
{
    robot::joystick_command_t joystick;

    float angularSlope = 0.0f;

    if (command.linear == 0.0f) {
        angularSlope = (command.angular < 0) ? calibration.turnInPlaceNegativeAngularVelocityRatio
                                             : calibration.turnInPlacePositiveAngularVelocityRatio;
    } else {
        angularSlope = (command.angular < 0) ? calibration.translatingNegativeAngularVelocityRatio
                                             : calibration.translatingPositiveAngularVelocityRatio;
    }

    if (command.linear > 0) {
        joystick.forward = command.linear / calibration.positiveLinearVelocityRatio + MIN_JOYSTICK_FORWARD;
    } else {
        joystick.forward = command.linear / calibration.negativeLinearVelocityRatio - MIN_JOYSTICK_FORWARD;
    }

    joystick.left = command.angular / angularSlope;
    joystick.gain = JOYSTICK_GAIN;

    return joystick;
}


bool InvacareATM::isJoystickCentered(const robot::joystick_command_t& joystick) const
{
    return (abs(joystick.forward) <= 1) && (abs(joystick.left) <= 1);
}


void InvacareATM::waitForMessages(void)
{
    FD_ZERO(&readSet);

    int joystickNfds = joystick.addToFDSet(readSet);
    int controllerNfds = controller.addToFDSet(readSet);

    select((joystickNfds > controllerNfds) ? joystickNfds : controllerNfds, &readSet, 0, 0, 0);
}


void InvacareATM::processMessages(bool canUseRobotCommand)
{
    if (joystick.isReady(readSet)) {
        handleJoystickMessage(canUseRobotCommand);
    }

    if (controller.isReady(readSet)) {
        handleControllerMessage();
    }
}


void InvacareATM::handleJoystickMessage(bool canUseRobotCommand)
{
    //     if(gotResponse > 0)
    //     {
    int bytesToRead = 10;

    joystick.read(joystickMessage, bytesToRead);

    if (joystickMessage[0] == 96) {
        //            std::cout<<"Joystick message:"<<(int)joystickMessage[0]<<' '<<(int)joystickMessage[1]<<'
        //            '<<(int)joystickMessage[2]<<' '<<(int)joystickMessage[3]<<'\n';
    }

    if (joystickMessage[0] == 96) {
        joystick.write(responseToJoystick, 6);
        // std::cerr << "responded to joystick msg\n";
    } else if (joystickMessage[0] == 116) {
        joystick.write(handshakeResponse, 4);
    } else {
        joystick.write(responseToJoystick, 6);
    }

    //  canUseRobotCommand = false;
    //	std::cout << "No Robot Command...\n";

    if (canUseRobotCommand) {
        convertJoystickCommandToJoystickMessage(robotJoystickCommand);
        controller.write(controlMessage, 10);

        //         std::cout << "mymsg = " << (unsigned short)controlMessage[1] << "\t" <<  (unsigned
        //         short)controlMessage[2] << "\t" <<  (unsigned short)controlMessage[3] << "\t" <<  (unsigned
        //         short)controlMessage[4] << "\t" <<  (unsigned short)controlMessage[8] << std::endl;
    } else {

        controller.write(joystickMessage, 10);
    }

    humanJoystickCommand.forward = joystickMessage[1] - 192;
    humanJoystickCommand.left = -(joystickMessage[2] - 192);
    humanJoystickCommand.gain = joystickMessage[3] - 128;
    humanJoystickCommand.timestamp = utils::system_time_us();
    //     }
    //     else
    //     {
    //         std::cerr << "Lost Communication...\n";
    //     }
}

void InvacareATM::handleControllerMessage(void)
{
    int bytesToRead = 6;
    controller.read(controllerMessage, bytesToRead);

    //     std::cout<<"Controller message:"<<(int)controllerMessage[0]<<'\n';

    if ((bytesToRead > 1) && (controllerMessage[0] == 33 || controllerMessage[0] == 5)) {
        gotResponse = 3;
    } else {
        //         std::cerr << "Controller did not respond\n";
        --gotResponse;
    }
}


void InvacareATM::convertJoystickCommandToJoystickMessage(const robot::joystick_command_t& joystick)
{
    utils::AutoMutex autoLock(messageLock);

    controlMessage[1] = joystick.forward + 192;
    controlMessage[2] = -joystick.left + 192;
    controlMessage[3] = joystick.gain + 128;

    // checksum equation
    int tmp = 283 + joystick.left - joystick.forward - joystick.gain;
    tmp = static_cast<unsigned char>(tmp) % 128u;

    // set checksum bytes
    unsigned char cs1 = tmp / 2;
    unsigned char cs2 = tmp - cs1;
    controlMessage[4] = cs1 | 128;
    controlMessage[8] = cs2 | 128;
}

}   // namespace robot
}   // namespace vulcan
