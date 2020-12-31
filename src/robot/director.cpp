/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of RobotInterfaceDirector.
*/

#include "robot/director.h"
#include "robot/params.h"
#include "robot/proximity_warning_indices.h"
#include "robot/motion_checker.h"
#include "robot/wheelchair.h"
#include "robot/commands.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"
#include <iostream>

// #define DEBUG_DIRECTOR_TIMING
// #define DEBUG_DIRECTOR
// #define DEBUG_WHEELCHAIR_COMMAND
// #define DEBUG_ISSUED_COMMAND

namespace vulcan
{
namespace robot
{

RobotInterfaceDirector::RobotInterfaceDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: TimeTriggeredDirector(load_command_period_ms(config))
, params(load_robot_controller_params(config))
, filter(params.filterParams)
, checker(create_motion_checker(params.checkerParams))
, wheelchair(create_wheelchair(params.driverParams))
, updatePeriodUs(params.commandPeriodMs*1000)
, previousOutputTimeUs(0)
{
    activeCommand.source = NO_SOURCE;
    
    startWheelchairThread();
}


RobotInterfaceDirector::~RobotInterfaceDirector(void)
{
    // For unique_ptr
}


void RobotInterfaceDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<motion_command_t>(this);
    communicator.subscribeTo<polar_laser_scan_t>(this);
}


system::UpdateStatus RobotInterfaceDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    motion_command_t     commandToIssue;    // command to be issued to the robot
    commanded_velocity_t velocityCommanded; // useful for display purposes
    commanded_joystick_t joystickCommanded;
    
#ifdef DEBUG_DIRECTOR_TIMING
    updateStartTimeUs = utils::system_time_us();
#endif
        
    dataLock.lock();
    
    proximity_warning_indices_t proximityIndices;
    
    
    // Sort the commands so they are considered in order of the most recent command first. The first valid
    // command is used. In the future, if there's an actual prioritization, then that would be the order in which
    // to consider the commands
    std::sort(commandQueue.begin(), commandQueue.end(), [](const motion_command_t& lhs, const motion_command_t& rhs)
                                                            {
                                                                return lhs.timestamp > rhs.timestamp;
                                                            });
    
    if((activeCommand.source != NO_SOURCE) && (activeCommand.source != ONBOARD_JOYSTICK))
    {
        commandQueue.push_back(activeCommand);
    }
    
    motion_command_t wheelchairMotionCommand = wheelchair->getWheelchairMotionCommand();
    commandQueue.push_front(wheelchairMotionCommand);   // always include the current joystick position from the wheelchair
    //         commandQueue.push_back(wheelchair->getWheelchairMotionCommand());   // always include the current joystick position from the wheelchair
    
#ifdef DEBUG_WHEELCHAIR_COMMAND
    std::cout<<"DEBUG DIRECTOR: Wheelchair command type: "<< wheelchairMotionCommand.commandType <<'\n';
    std::cout<<"DEBUG DIRECTOR: Wheelchair command timestamp: "<< wheelchairMotionCommand.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Wheelchair Linear  velocity command: "<< wheelchairMotionCommand.velocityCommand.linear <<" m/s\n";
    std::cout<<"DEBUG DIRECTOR: Wheelchair Angular velocity command: "<< wheelchairMotionCommand.velocityCommand.angular <<" rad/s\n";
    std::cout<<"DEBUG DIRECTOR: Wheelchair Linear velocity command timestamp: "<< wheelchairMotionCommand.velocityCommand.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Wheelchair Joystick forward: "<< wheelchairMotionCommand.joystickCommand.forward <<"\n";
    std::cout<<"DEBUG DIRECTOR: Wheelchair Joystick left: "   << wheelchairMotionCommand.joystickCommand.left <<"\n";
    std::cout<<"DEBUG DIRECTOR: Wheelchair Joysitck command timestamp: "<< wheelchairMotionCommand.joystickCommand.timestamp <<'\n';
#endif
    
    commandToIssue = selectMotionCommand();
    
#ifdef DEBUG_ISSUED_COMMAND
    std::cout<<"DEBUG DIRECTOR: Selected command type: "<< commandToIssue.commandType <<'\n';
    std::cout<<"DEBUG DIRECTOR: Selected command timestamp: "<< commandToIssue.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Selected linear  velocity command: "<< commandToIssue.velocityCommand.linear <<" m/s\n";
    std::cout<<"DEBUG DIRECTOR: Selected angular velocity command: "<< commandToIssue.velocityCommand.angular <<" rad/s\n";
    std::cout<<"DEBUG DIRECTOR: Selected velocity command timestamp: "<< commandToIssue.velocityCommand.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Selected joystick forward: "<< commandToIssue.joystickCommand.forward <<"\n";
    std::cout<<"DEBUG DIRECTOR: Selected joystick left: "   << commandToIssue.joystickCommand.left <<"\n";
    std::cout<<"DEBUG DIRECTOR: Selected joystick command timestamp: "<< commandToIssue.joystickCommand.timestamp <<"\n\n";
#endif
    
    convertLaserDataToScanPoints();
    
    if(commandToIssue.source != ONBOARD_JOYSTICK)
    {
        // FIXME: temporarily turned-off for initial run
        //             // proximity checker in action. NOTE: Shuold I be using joystick command for this? 
        //             velocity_command_t safeVelocityCommand = checker->adjustCommandIfNeeded(commandToIssue.velocityCommand, scanPoints, proximityIndices);
        //             if(checker->hasAdjustedCommand())
        //             {
        //                 commandToIssue.velocityCommand = safeVelocityCommand;
        //                 commandToIssue.commandType     = VELOCITY;
        //             }
        
#ifdef DEBUG_DIRECTOR
        std::cout<<"DEBUG DIRECTOR: Setting motor command...\n";
#endif
        
        wheelchair->setMotorCommandTarget(commandToIssue); // issuing command to the robot
    }
    
    activeCommand = commandToIssue;
    commandQueue.clear();
    dataLock.unlock();
    
    velocityCommanded = wheelchair->updateCommandedVelocities();
    joystickCommanded = wheelchair->updateCommandedJoystick();
    
    communicator.sendMessage(joystickCommanded);
    communicator.sendMessage(velocityCommanded);
    communicator.sendMessage(proximityIndices);

    // The robot_interface is always running
    return system::UpdateStatus::running;
}


void RobotInterfaceDirector::shutdown(system::ModuleCommunicator& communicator)
{
    // TODO: Graceful shut down
}


// Handlers for incoming data to be distributed to the various modules
void RobotInterfaceDirector::handleData(const motion_command_t& command, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    commandQueue.push_back(command);
    
#ifdef DEBUG_DIRECTOR
    std::cout<<"DEBUG DIRECTOR: Received command type: "<< command.commandType <<'\n';
    std::cout<<"DEBUG DIRECTOR: Received command timestamp: "<< command.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Received linear  velocity command: "<< command.velocityCommand.linear <<" m/s\n";
    std::cout<<"DEBUG DIRECTOR: Received angular velocity command: "<< command.velocityCommand.angular <<" rad/s\n";
    std::cout<<"DEBUG DIRECTOR: Received velocity command timestamp: "<< command.velocityCommand.timestamp <<'\n';
    std::cout<<"DEBUG DIRECTOR: Received joystick forward: "<< command.joystickCommand.forward <<"\n";
    std::cout<<"DEBUG DIRECTOR: Received joystick left: "   << command.joystickCommand.left <<"\n";
    std::cout<<"DEBUG DIRECTOR: Received joystick command timestamp: "<< command.joystickCommand.timestamp <<"\n\n";
#endif

}


void RobotInterfaceDirector::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    // Find the appropriate channel on which this laser data arrived
    for(auto laserIt = laserScans.begin(), laserEnd = laserScans.end(); laserIt != laserEnd; ++laserIt)
    {
        if(laserIt->channel == channel)
        {
            laserIt->scan = scan;
            return;
        }
    }

    // If here, then this channel is a new stream of laser data, so create a new laser_data_t structure for it
    laser_data_t data = {channel, scan};

    laserScans.push_back(data);
}


void RobotInterfaceDirector::startWheelchairThread(void)
{
    wheelchairThread.attachTask(wheelchair.get());
    wheelchairThread.start();
}


motion_command_t RobotInterfaceDirector::selectMotionCommand(void)
{
    // The command to select is the most recent motor command from the active source
    // Commands are queued, so start at the end and go until a command is found that satisfies the filter
    // If no commands are found, then the correct thing is to not move, so a stop/(0,0) command is issued

    // creating a stop command for the case when no command is valid in the command queue
    velocity_command_t zeroVelocity(0.0, 0.0, utils::system_time_us());
    joystick_command_t centeredJoystick(0, 0, 100);
    centeredJoystick.timestamp = utils::system_time_us();

    motion_command_t commandToIssue(NO_SOURCE, zeroVelocity, centeredJoystick);
    commandToIssue.timestamp   = utils::system_time_us();
    
#ifdef DEBUG_DIRECTOR
    std::cout<<"DEBUG DIRECTOR: Validating command cues...\n";
#endif
    
    // find valid command in the queue and return
    for(size_t i = 0; i < commandQueue.size(); ++i)
    {
        if(filter.validMotionCommand(commandQueue[i]))
        {
            
#ifdef DEBUG_DIRECTOR
            std::cout<<"DEBUG DIRECTOR: Command validated.\n";
#endif
            
            commandToIssue = commandQueue[i];
            break;
        }
    }
    
    return commandToIssue;
}


void RobotInterfaceDirector::convertLaserDataToScanPoints(void)
{
    scanPoints.clear();

    for(auto laserIt = laserScans.begin(), laserEnd = laserScans.end(); laserIt != laserEnd; ++laserIt)
    {
        polar_scan_to_cartesian_scan_in_robot_frame(laserIt->scan, cartesianScan, true);

        scanPoints.insert(scanPoints.end(), cartesianScan.scanPoints.begin(), cartesianScan.scanPoints.end());
    }
}

} // namespace robot
} // namespace vulcan
