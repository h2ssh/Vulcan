/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     quantum_6000.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of Quantum6000.
*/

#include <robot/quantum_6000.h>
#include <utils/auto_mutex.h>
#include <utils/timestamp.h>
#include <sys/select.h>
#include <libpcan.h>
#include <iostream>
#include <iomanip>
#include <cmath>


// #define DEBUG_PORT_ASSIGNMENT
// #define DEBUG_JOYSTICK_MESSAGE_FLOW
// #define DEBUG_CONTROLLER_MESSAGE_FLOW
// #define DEBUG_JOYSTICK_VALUES
// #define DEBUG_MESSAGES
// #define DEBUG_JOYSTICK_RATE
// #define DEBUG_JOYSTICK_CHANGE

namespace vulcan
{
namespace robot
{

const uint32_t JOYSTICK_POSITION_MESSAGE_ID = 0x81;

const uint8_t JOYSTICK_X_INDEX    = 0;
const uint8_t JOYSTICK_Y_INDEX    = 1;
const uint8_t JOYSTICK_GAIN_INDEX = 3;
const uint8_t JOYSTICK_CENTERED_INDEX = 5;

const float POSITIVE_TURN_IN_PLACE_SLOPE = 0.0554f*0.625f;
const float NEGATIVE_TURN_IN_PLACE_SLOPE = 0.0531f*0.625f;

const float POSITIVE_ANGULAR_VELOCITY_SLOPE = 0.053f;
const float NEGATIVE_ANGULAR_VELOCITY_SLOPE = 0.046f;

const float LINEAR_VELOCITY_SLOPE  = 0.0287f;

// const int8_t MIN_JOYSTICK_X = 10; // obsolete deadband parameters
// const int8_t MIN_JOYSTICK_Y = 5;

// Currently, always using the max gain for control
const char JOYSTICK_GAIN               = 0x64;
const char JOYSTICK_CENTERED_VALUE     = 0x07;
const char JOYSTICK_NOT_CENTERED_VALUE = 0x03;

const char MAX_JOYSTICK = 100;
const char MIN_JOYSTICK = -100;


bool is_joystick_position_message(const TPCANMsg& message);

char  convert_velocity_to_joystick_position(float velocity, float velocity_scale);
float convert_joystick_position_to_velocity(char  position, float velocity_scale);
char  clamp_joystick_position(int position);

void convert_joystick_command_to_can_message(const joystick_command_t& joystick, TPCANMsg& message);
joystick_command_t convert_can_message_to_joystick_command(const TPCANMsg& message);

void display_message_counts(const std::map<uint8_t, int>& joystickMessages, const std::map<uint8_t, int>& controllerMessages);
void print_read_message(const TPCANRdMsg& message);
void print_message(const TPCANMsg& message);


Quantum6000::Quantum6000(const std::string& controllerPort, const std::string& joystickPort)
    : lastDisplayTime(0)
    , joystickBus(new utils::CanBus(joystickPort, CAN_BAUD_250K))
    , controllerBus(new utils::CanBus(controllerPort, CAN_BAUD_250K))
    , firstJoystickMessageTime(0)
    , haveReceivedPositionMessage(false)
{
    calibration.positiveLinearVelocityRatio = LINEAR_VELOCITY_SLOPE;
    calibration.negativeLinearVelocityRatio = LINEAR_VELOCITY_SLOPE;

    calibration.translatingPositiveAngularVelocityRatio = POSITIVE_ANGULAR_VELOCITY_SLOPE;
    calibration.translatingNegativeAngularVelocityRatio = NEGATIVE_ANGULAR_VELOCITY_SLOPE;

    calibration.turnInPlacePositiveAngularVelocityRatio = POSITIVE_TURN_IN_PLACE_SLOPE;
    calibration.turnInPlaceNegativeAngularVelocityRatio = NEGATIVE_TURN_IN_PLACE_SLOPE;

    init();
}


Quantum6000::Quantum6000(const wheelchair_joystick_calibration_t& calibration, const std::string& controllerPort, const std::string& joystickPort)
    : lastDisplayTime(0)
    , joystickBus(new utils::CanBus(joystickPort, CAN_BAUD_250K))
    , controllerBus(new utils::CanBus(controllerPort, CAN_BAUD_250K))
    , firstJoystickMessageTime(0)
    , haveReceivedPositionMessage(false)
    , calibration(calibration)
{
    init();
}


Quantum6000::~Quantum6000(void)
{
    delete controllerBus;
    delete joystickBus;
}


void Quantum6000::init(void)
{
    std::cerr << "Initializing CAN-Buses, will take a moment.\n";

    std::cerr << "Opened Joystick CAN-Bus\n";
    std::cerr << "Opened Controller CAN-Bus\n";

    std::cerr << "CAN-Buses initialized. Please turn Quantum6000 on now.\n";
    std::cerr << "Quantum6000 initialized successfully.\n";

    currentDataTimestamp = utils::system_time_us();

    joystickFd   = joystickBus->getFileHandle();
    controllerFd = controllerBus->getFileHandle();
}


commanded_velocity_t Quantum6000::convertJoystickCommandToCommandedVelocity(const joystick_command_t& joystick) const
{
    // the output should be useful for display purposes. Don't trust the values returned by this function too much.
    commanded_velocity_t command;

    float TURN_RATE_IN_PLACE  = 0.45;
    float TURN_RATE_BASE      = 0.45;
    float TURN_REDUCTION_RATE = 0.006;

    float MOTOR_MU    = 0.26;
    float MOTOR_BETA  = 5.5;
    float MOTOR_GAMMA = 0.165*MOTOR_BETA/MOTOR_MU;
    float MOTOR_ALPHA = 0.026*MOTOR_BETA;

    float VEHICLE_WHEELBASE = 0.603;

    float lateralCommandScaling;
    if(joystick.forward == 0)
    {
        lateralCommandScaling = TURN_RATE_IN_PLACE;
    }
    else
    {
        lateralCommandScaling = TURN_RATE_BASE*(1 - TURN_REDUCTION_RATE*std::abs(joystick.forward));
    }

    float rightMotorCommand = joystick.forward + lateralCommandScaling*(joystick.left);
    float leftMotorCommand  = joystick.forward - lateralCommandScaling*(joystick.left);

    float rightMotorSpeedSteadyState;
    float leftMotorSpeedSteadyState;
    if(rightMotorCommand > MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA)
    {
        rightMotorSpeedSteadyState = MOTOR_ALPHA/MOTOR_BETA*rightMotorCommand - MOTOR_MU*MOTOR_GAMMA/MOTOR_BETA;
    }
    else if(rightMotorCommand < -MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA)
    {
        rightMotorSpeedSteadyState = MOTOR_ALPHA/MOTOR_BETA*rightMotorCommand + MOTOR_MU*MOTOR_GAMMA/MOTOR_BETA;
    }
    else
    {
        rightMotorSpeedSteadyState = 0;
    }

    if(leftMotorCommand > MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA)
    {
        leftMotorSpeedSteadyState = MOTOR_ALPHA/MOTOR_BETA*leftMotorCommand - MOTOR_MU*MOTOR_GAMMA/MOTOR_BETA;
    }
    else if(rightMotorCommand < -MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA)
    {
        leftMotorSpeedSteadyState = MOTOR_ALPHA/MOTOR_BETA*leftMotorCommand + MOTOR_MU*MOTOR_GAMMA/MOTOR_BETA;
    }
    else
    {
        leftMotorSpeedSteadyState = 0;
    }

    command.linearVelocity  = 0.5*(rightMotorSpeedSteadyState + leftMotorSpeedSteadyState);
    command.angularVelocity = 1/VEHICLE_WHEELBASE*(rightMotorSpeedSteadyState - leftMotorSpeedSteadyState);
    command.timestamp       = joystick.timestamp;

    return command;
}


joystick_command_t Quantum6000::convertVelocityCommandToJoystickCommand(const velocity_command_t& command) const
{
    // FIXME: hard coded parameters for feedforward controller
    float TURN_RATE_IN_PLACE  = 0.45;
    float TURN_RATE_BASE      = 0.45;
    float TURN_REDUCTION_RATE = 0.006;

    float MOTOR_MU    = 0.26;
    float MOTOR_BETA  = 5.5;
    float MOTOR_GAMMA = 0.165/MOTOR_MU;
    float MOTOR_ALPHA = 0.026*MOTOR_BETA;

    float VEHICLE_WHEELBASE = 0.603;

    // conveting vehicle velocity to wheel speeds
    float desiredRightWheelSpeed = command.linear + (VEHICLE_WHEELBASE/2.0)*command.angular;
    float desiredLeftWheelSpeed  = command.linear - (VEHICLE_WHEELBASE/2.0)*command.angular;
    float rightMotorCommand = 0.0f;
    float leftMotorCommand  = 0.0f;

    // wheel speeds to motor commands
    if(fabs(desiredRightWheelSpeed) < 0.005) // if desired speed steady state is very small
    {
        rightMotorCommand = 0.0f;
    }
    else
    {
        rightMotorCommand = MOTOR_BETA/MOTOR_ALPHA*desiredRightWheelSpeed + copysign(MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA, desiredRightWheelSpeed);
    }

    if(fabs(desiredLeftWheelSpeed) < 0.005) // if desired speed steady state is very small
    {
        leftMotorCommand = 0.0f;
    }
    else
    {
        leftMotorCommand = MOTOR_BETA/MOTOR_ALPHA*desiredLeftWheelSpeed + copysign(MOTOR_MU*MOTOR_GAMMA/MOTOR_ALPHA, desiredLeftWheelSpeed);
    }

    // motor commands to joystick commands
    int16_t joystickForward = static_cast<int16_t>((rightMotorCommand + leftMotorCommand)/2.0);

    float turnRate = (joystickForward == 0) ? TURN_RATE_IN_PLACE : TURN_RATE_BASE - TURN_REDUCTION_RATE*(fabs(float(joystickForward)));
    int16_t joystickLeft = static_cast<int16_t>(rightMotorCommand - leftMotorCommand)/2.0/turnRate;

//     std::cout<<"Motion:"<<command.linearVelocity<<','<<command.angularVelocity<<" Joystick:"<<joystick.x<<','<<joystick.y<<'\n';

    return joystick_command_t(clamp_joystick_position(joystickForward), clamp_joystick_position(joystickLeft), JOYSTICK_GAIN);
}


bool Quantum6000::isJoystickCentered(const joystick_command_t& joystick) const
{
    return (joystick.forward == 0) && (joystick.left == 0);
}


void Quantum6000::waitForMessages(void)
{
    int highestFd = (joystickFd > controllerFd) ? joystickFd : controllerFd;
    ++highestFd; // nfds for select wants highest fd in the set + 1

    FD_ZERO(&readSet);
    FD_SET(joystickFd, &readSet);
    FD_SET(controllerFd, &readSet);

    select(highestFd, &readSet, 0, 0, 0);
}


void Quantum6000::processMessages(bool canUseRobotCommand)
{
    /*
    * The controller and the joystick send messages back and forth. This driver intercepts those messages
    * and either injects a robot message, or passes the message on. At the current time, only the joystick
    * message that issues a motor command is replaced.
    */

    if(FD_ISSET(joystickFd, &readSet))
    {
        handleJoystickBus(canUseRobotCommand);
    }

    if(FD_ISSET(controllerFd, &readSet))
    {
        #ifdef DEBUG_CONTROLLER_MESSAGE_FLOW
        std::cout<<"INFO: Quantum6000::readCanBusses: Received controller message\n";
        #endif

        handleControllerBus();
    }

#ifdef DEBUG_MESSAGES
    if(utils::system_time_us() - lastDisplayTime > 5000000)
    {
        display_message_counts(joystickMessageCounts, controllerMessageCounts);
    }
#endif
}


void Quantum6000::handleJoystickBus(bool canUseRobotCommand)
{
    joystickBus->readMessageTime(internalJoystickMessage);

    utils::AutoMutex autoLock(messageLock);

    ++joystickMessageCounts[internalJoystickMessage.Msg.ID];

    if(!firstJoystickMessageTime)
    {
        firstJoystickMessageTime = utils::system_time_us();
    }

    if(is_joystick_position_message(internalJoystickMessage.Msg))
    {
        if(!haveReceivedPositionMessage)
        {
            std::cout<<"INFO:Quantum6000: Received initial joystick position message.\n";
        }

        humanJoystickCommand        = convert_can_message_to_joystick_command(internalJoystickMessage.Msg);
        haveReceivedPositionMessage = true;
    }

    // Only send the override if the joystick is centered. If the joystick
    // isn't centered, then the human is actively driving the wheelchair,
    // so don't override that
    // NOTE: This process needs to change. The driver commands need to go out to the world,
    //       and then return because the safety criterion needs to be applied.
    if(is_joystick_position_message(internalJoystickMessage.Msg) && canUseRobotCommand)
    {
        convert_joystick_command_to_can_message(robotJoystickCommand, externalJoystickMessage);

        controllerBus->sendMessage(externalJoystickMessage);

        humanIsDriver = false;

#ifdef DEBUG_JOYSTICK_MESSAGE_FLOW
        std::cout<<"INFO: Quantum6000::handleJoystickBus: Sent external joystick message:\n";
        print_message(externalJoystickMessage);
        std::cout<<std::endl;
#endif
    }
    else
    {
        if(is_joystick_position_message(internalJoystickMessage.Msg))
        {
            humanIsDriver = true;

#ifdef DEBUG_JOYSTICK_MESSAGE_FLOW
            std::cout<<"INFO: Quantum6000::handleJoystickBus: Sent internal joystick message:\n";
            print_read_message(internalJoystickMessage);
            std::cout<<'\n';
#endif
        }

        controllerBus->sendMessage(internalJoystickMessage.Msg);
    }

#ifdef DEBUG_JOYSTICK_VALUES
    if(is_joystick_position_message(internalJoystickMessage.Msg))
    {
        std::cout<<"Joystick: Human:("<<humanJoystickCommand.forward<<','<<humanJoystickCommand.left<<','<<humanJoystickCommand.gain<<")\n"
                 <<"          Robot:("<<robotJoystickCommand.forward<<','<<robotJoystickCommand.left<<','<<robotJoystickCommand.gain<<")\n";
    }
#endif
}


void Quantum6000::handleControllerBus(void)
{
    ++controllerMessageCounts[controllerMessage.Msg.ID];

    controllerBus->readMessageTime(controllerMessage);
    joystickBus->sendMessage(controllerMessage.Msg);

#ifdef DEBUG_CONTROLLER_MESSAGE_FLOW
    std::cout<<"INFO: Quantum6000::handleControllerBus: Sent controller message:\n";
    print_read_message(controllerMessage);
    std::cout<<std::endl;
#endif
}


bool is_joystick_position_message(const TPCANMsg& message)
{
    return message.ID == JOYSTICK_POSITION_MESSAGE_ID;
}


char convert_velocity_to_joystick_position(float velocity, float velocity_scale) // now obsolete
{
    int position = static_cast<int>(velocity / velocity_scale);

//     if((velocity > 0 && position < 0) ||
//        (velocity < 0 && position > 0))
//     {
//         position = 0;
//     }

    return clamp_joystick_position(position);
}


float convert_joystick_position_to_velocity(char position, float velocity_scale) // now obsolete
{
    float velocity = position * velocity_scale;

//     if((position > 0 && velocity < 0) ||
//        (position < 0 && velocity > 0))
//     {
//         velocity = 0;
//     }

    return velocity;
}


char clamp_joystick_position(int position)
{
    if(position > MAX_JOYSTICK)
    {
        position = MAX_JOYSTICK;
    }

    if(position < MIN_JOYSTICK)
    {
        position = MIN_JOYSTICK;
    }

    return position;
}


void convert_joystick_command_to_can_message(const joystick_command_t& joystick, TPCANMsg& message)
{
    message.ID      = 0x081;
    message.MSGTYPE = MSGTYPE_STANDARD;
    message.LEN     = 8;
    message.DATA[2] = 0x81;
    message.DATA[4] = 0x03;
    message.DATA[6] = 0x00;
    message.DATA[7] = 0x00;

    message.DATA[JOYSTICK_X_INDEX]    = -joystick.left;
    message.DATA[JOYSTICK_Y_INDEX]    = joystick.forward;
    message.DATA[JOYSTICK_GAIN_INDEX] = JOYSTICK_GAIN;

    if(joystick.forward == 0 && joystick.left == 0)
    {
        message.DATA[JOYSTICK_CENTERED_INDEX] = JOYSTICK_CENTERED_VALUE;
    }
    else
    {
        message.DATA[JOYSTICK_CENTERED_INDEX] = JOYSTICK_NOT_CENTERED_VALUE;
    }
}


joystick_command_t convert_can_message_to_joystick_command(const TPCANMsg& message)
{
    joystick_command_t joystick;

    joystick.timestamp = utils::system_time_us();

    joystick.left    = -static_cast<char>(message.DATA[JOYSTICK_X_INDEX]);
    joystick.forward = static_cast<char>(message.DATA[JOYSTICK_Y_INDEX]);
    joystick.gain    = static_cast<char>(message.DATA[JOYSTICK_GAIN_INDEX]);

    return joystick;
}


void display_message_counts(const std::map<uint8_t, int>& joystickMessages, const std::map<uint8_t, int>& controllerMessages)
{
    std::cout<<"DEBUG:Quantum6000: Joystick message counts:\n";
    for(auto joyIt = joystickMessages.begin(), joyEnd = joystickMessages.end(); joyIt != joyEnd; ++joyIt)
    {
        std::cout<<(unsigned int)joyIt->first<<" : "<<joyIt->second<<'\n';
    }

    std::cout<<"DEBUG:Quantum6000: Controller message counts:\n";
    for(auto controlIt = controllerMessages.begin(), controlEnd = controllerMessages.end(); controlIt != controlEnd; ++controlIt)
    {
        std::cout<<(unsigned int)controlIt->first<<" : "<<controlIt->second<<'\n';
    }
}


void print_read_message(const TPCANRdMsg& message)
{
    print_message(message.Msg);
}

void print_message(const TPCANMsg& message)
{
    std::cout << (int)message.ID << '\t'
              << std::dec
              << (short)message.MSGTYPE << '\t'
              << (short)message.LEN << '\t'
              << (short)message.DATA[0] << '\t'
              << (short)message.DATA[1] << '\t'
              << (short)message.DATA[2] << '\t'
              << (short)message.DATA[3] << '\t'
              << (short)message.DATA[4] << '\t'
              << (short)message.DATA[5] << '\t'
              << (short)message.DATA[6] << '\t'
              << (short)message.DATA[7] << '\n';
}

} // namespace drivers
} // namespace vulcan
