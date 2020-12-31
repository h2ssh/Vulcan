/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     commands.h
* \author   Collin Johnson and Jong Jin Park
* 
* Declaration of types related to commanding the robot to move:
*
*   - commanded_velocity_t
*   - commanded_joystick_t
*   - command_type_t 
*   - command_source_t
*   - velocity_command_t
*   - joystick_command_t
*   - motion_commnad_t
*/

#ifndef CONTROL_COMMANDS_H
#define CONTROL_COMMANDS_H

#include <cstdint>
#include "system/message_traits.h"
#include <cereal/access.hpp>

namespace vulcan
{

namespace robot
{

// command status
/**
* commanded_velocity_t defines the velocity command most recently sent to the robot's motors.
* The command will be related to the most recent motion command, but might not be exactly the
* same due to the internal characteristics of the robot controller being used.
*/
struct commanded_velocity_t
{
    int64_t timestamp;
    
    float linearVelocity;
    float angularVelocity;
        
    commanded_velocity_t(float linear = 0, float angular = 0, int64_t timestamp = 0)
    : timestamp(timestamp)
    , linearVelocity(linear)
    , angularVelocity(angular)
    {    
    }

};

/**
* commanded_joystick_t defines the joystick command most recently sent to the robot's motors.
* The command will be related to the most recent motion command, but might not be exactly the
* same due to the internal characteristics of the robot controller being used.
*/
struct commanded_joystick_t
{
    int64_t timestamp;
    
    int16_t forward;
    int16_t left;
        
    commanded_joystick_t(int16_t forward = 0, int16_t left = 0, int64_t timestamp = 0)
    : timestamp(timestamp)
    , forward(forward)
    , left(left)
    {
    }

};


// actual command
/**
* command_type_t defines all possible types of motion commands to the robot interface.
*/
enum command_type_t
{
    VELOCITY,
    JOYSTICK
};

/**
* command_source_t defines all possible sources of motor commands. Identifying the source
* of a command allows it to be filtered depending on the current control source of the
* robot.
*/
enum command_source_t
{
    ONBOARD_JOYSTICK,
    REMOTE_JOYSTICK,
    AUTONOMOUS_CONTROLLER,
    ANY_SOURCE,
    NO_SOURCE
};

/**
* velocity_command_t defines a set target linear and angular velocity sent to the robot interface.
*/
struct velocity_command_t
{
    int64_t timestamp;
    
    float linear;
    float angular;
    
    velocity_command_t(float linear = 0, float angular = 0, int64_t timestamp = 0)
    : timestamp(timestamp)
    , linear(linear)
    , angular(angular)
    {
    }
};

/**
* joystick_command_t defines a joystick command issued from a joystick controller for the
* robot. In the case of the wheelchair, the joystick is tightly coupled to the motion of
* the wheelchair, and the wheelchair produces joystick commands issued by the human driver.
*/
struct joystick_command_t
{
    int64_t timestamp;
    
    int16_t forward;
    int16_t left;
    int16_t gain; // max gain = 100
    
    joystick_command_t(int16_t forward = 0, int16_t left = 0, int16_t gain = 100, int64_t timestamp = 0)
    : timestamp(timestamp)
    , forward(forward)
    , left(left)
    , gain(gain)
    {
    }
};

/**
* motion_command_t defines a set of parameters for the robot controller to use in moving
* the robot around the world. A motion command is either a velocity command or joystic
* positions.
*/
struct motion_command_t
{
    int64_t timestamp;
    
    command_type_t   commandType;
    command_source_t source;
    
    velocity_command_t velocityCommand;
    joystick_command_t joystickCommand;
    
    motion_command_t(void)
    : commandType(VELOCITY)
    , source(NO_SOURCE) // default source to NO_SOURCE, so that the robot does not use the command unless the source is properly initialized.
    , velocityCommand(0,0,0)
    , joystickCommand(0,0,100,0)
    {
    }
    
    motion_command_t(command_source_t source, const velocity_command_t& velocityCommand)
    : timestamp(velocityCommand.timestamp)
    , commandType(VELOCITY)
    , source(source)
    , velocityCommand(velocityCommand)
    , joystickCommand(0,0,100,0)
    {
    }

    motion_command_t(command_source_t source, const joystick_command_t& joystickCommand)
    : timestamp(joystickCommand.timestamp)
    , commandType(JOYSTICK)
    , source(source)
    , velocityCommand(0,0,0)
    , joystickCommand(joystickCommand)
    {
    }

    motion_command_t(command_source_t source, const velocity_command_t& velocityCommand, const joystick_command_t& joystickCommand)
    : timestamp(joystickCommand.timestamp)
    , commandType(JOYSTICK) // if both commands are coming in use lower level command by default
    , source(source)
    , velocityCommand(velocityCommand)
    , joystickCommand(joystickCommand)
    {
    }
};


// Serialization support
// template <class Archive>
// void serialize(Archive& ar, commanded_velocity_t& commanded)
// {
//     ar (commanded.timestamp,
//         commanded.linearVelocity,
//         commanded.angularVelocity);
// }
// 
// template <class Archive>
// void serialize(Archive& ar, commanded_joystick_t& commanded)
// {
//     ar (commanded.timestamp,
//         commanded.forward,
//         commanded.left);
// }

template <class Archive>
void serialize(Archive& ar, velocity_command_t& command)
{
    ar (command.timestamp,
        command.linear,
        command.angular);
}

template <class Archive>
void serialize(Archive& ar, joystick_command_t& command)
{
    ar (command.timestamp,
        command.forward,
        command.left,
        command.gain);
}

template <class Archive>
void serialize(Archive& ar, motion_command_t& motionCommand)
{
    ar (motionCommand.timestamp,
        motionCommand.commandType,
        motionCommand.source,
        motionCommand.velocityCommand,
        motionCommand.joystickCommand);
}

} // robot
} // vulcan

DEFINE_SYSTEM_MESSAGE(robot::motion_command_t, ("ROBOT_MOTION_COMMAND"))

#endif // CONTROL_COMMANDS_H
