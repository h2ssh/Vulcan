/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wheelchair.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of create_wheelchair factory.
*/

#include "robot/wheelchair.h"
#include "robot/quantum_6000.h"
#include "robot/invacare_atm.h"
#include "robot/params.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace robot
{

// If the motion command is this old, then switch back to joystick control. Don't want to
// use stale commands.
const int64_t COMMAND_TIMEOUT_US = 500000;

std::unique_ptr<Wheelchair> create_wheelchair(const wheelchair_params_t& params)
{
    if(params.wheelchairType == robot::QUANTUM_6000_TYPE)
    {
        return std::unique_ptr<Wheelchair>(new robot::Quantum6000(params.calibration,
                                                                  params.controllerPort,
                                                                  params.joystickPort));
    }
    else if(params.wheelchairType == robot::INVACARE_ATM_TYPE)
    {
        return std::unique_ptr<Wheelchair>(new robot::InvacareATM(params.calibration,
                                                                  params.joystickPort,
                                                                  params.controllerPort));
    }

    std::cerr<<"ERROR: createWheelchair: No valid wheelchair type specified: "<<params.wheelchairType<<std::endl;
    assert(false);

    return std::unique_ptr<Wheelchair>();
}


Wheelchair::Wheelchair(void)
    : humanIsDriver(true)
{

}


void Wheelchair::setMotorCommandTarget(const motion_command_t& command)
{
    utils::AutoMutex autoLock(commandLock);

    commandRecieved = command;
    
    // Only care about the time from when the command is received for the timeout.
    // Furthermore, this ensures that the timestamp is meaningful.
    commandRecieved.timestamp = utils::system_time_us();
    
    if(commandRecieved.commandType == VELOCITY)
    {
        robotJoystickCommand = convertVelocityCommandToJoystickCommand(commandRecieved.velocityCommand);
        robotJoystickCommand.timestamp = utils::system_time_us();
    }
    else if(commandRecieved.commandType == JOYSTICK)
    {
        robotJoystickCommand = commandRecieved.joystickCommand;
        robotJoystickCommand.left -= 2;
        robotJoystickCommand.timestamp = utils::system_time_us();
    }
    else
    {
        std::cout<<"ERROR:Unknown command type.\n";
    }
    
//    std::cout<<"WHEELCHAIR: Setting robot joystick command: ("<<robotJoystickCommand.forward<<','<<robotJoystickCommand.left<<")\n";
}


commanded_velocity_t Wheelchair::updateCommandedVelocities(void)
{
    utils::AutoMutex autoLock(commandLock);

    commanded_velocity_t commandedVelocity;

    if(humanIsDriver)
    {
        commandedVelocity = convertJoystickCommandToCommandedVelocity(humanJoystickCommand);
    }
    else
    {
        commandedVelocity = convertJoystickCommandToCommandedVelocity(robotJoystickCommand);
    }

    commandedVelocity.timestamp = utils::system_time_us();

    return commandedVelocity;
}


commanded_joystick_t Wheelchair::updateCommandedJoystick(void)
{
    utils::AutoMutex autoLock(commandLock);
    
    commanded_joystick_t commandedJoystick;
    
    if(humanIsDriver)
    {
        commandedJoystick = commanded_joystick_t(humanJoystickCommand.forward, humanJoystickCommand.left, humanJoystickCommand.timestamp);
    }
    else
    {
        commandedJoystick = commanded_joystick_t(robotJoystickCommand.forward, robotJoystickCommand.left, robotJoystickCommand.timestamp);
    }
    
    return commandedJoystick;
}



motion_command_t Wheelchair::getWheelchairMotionCommand(void) const
{
    utils::AutoMutex autoLock(commandLock);
    
//    std::cout<<"WHEELCHAIR: Human joystick command: ("<<humanJoystickCommand.x<<','<<humanJoystickCommand.y<<")\n";
    
    commanded_velocity_t commandedVelocity = convertJoystickCommandToCommandedVelocity(humanJoystickCommand);
    velocity_command_t   velocityCommand(commandedVelocity.linearVelocity, commandedVelocity.angularVelocity, humanJoystickCommand.timestamp);
    motion_command_t     motionCommand(ONBOARD_JOYSTICK, velocityCommand, humanJoystickCommand);
        
    return motionCommand;
}

velocity_command_t Wheelchair::getWheelchairVelocityCommand(void) const
{
    utils::AutoMutex autoLock(commandLock);

    commanded_velocity_t commandedVelocity = convertJoystickCommandToCommandedVelocity(humanJoystickCommand);
    velocity_command_t   velocityCommand(commandedVelocity.linearVelocity, commandedVelocity.angularVelocity, humanJoystickCommand.timestamp);
    
    return velocityCommand;
}


joystick_command_t Wheelchair::getJoystickCommand(void) const
{
    utils::AutoMutex autoLock(commandLock);

    return humanJoystickCommand;
}


void Wheelchair::setJoystickCommand(const joystick_command_t& command)
{
    utils::AutoMutex autoLock(commandLock);

    robotJoystickCommand      = command;
    commandRecieved.timestamp = command.timestamp;
}


int Wheelchair::run(void)
{
    while(true)
    {
        waitForMessages();

        bool canUseRobotCommand = isJoystickCentered(humanJoystickCommand) && (utils::system_time_us() - commandRecieved.timestamp < COMMAND_TIMEOUT_US);
	
        humanIsDriver = !isJoystickCentered(humanJoystickCommand);

        processMessages(canUseRobotCommand);
    }

    return 0;
}

} // namespace robot
} // namespace vulcan
