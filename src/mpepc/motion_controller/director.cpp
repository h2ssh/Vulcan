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
* Definition of the MotionControllerDirector.
*/

#include "mpepc/motion_controller/director.h"
#include "mpepc/motion_controller/messages.h"
#include "mpepc/motion_controller/task/task.h"
#include "mpepc/motion_controller/data.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"
#include <iostream>

namespace vulcan
{

namespace mpepc
{

MotionControllerDirector::MotionControllerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: params_(load_motion_controller_params(config))
, haveState_(false)
, isPaused_(false)
, isCancelled_(false)
, cancelledTimestamp_(0)
, controllers_(params_.controllerTypes, params_)
, dataTrigger_(false)
{
}


void MotionControllerDirector::subscribeToData(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<std::shared_ptr<MotionControllerTask>>(this);
    producer.subscribeTo<motion_state_t>(this);
    producer.subscribeTo<motion_controller_command_message_t>(this);
}


system::TriggerStatus MotionControllerDirector::waitForTrigger(void)
{
    return !dataTrigger_.timedWait(100) ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus MotionControllerDirector::runUpdate(system::ModuleCommunicator& transmitter)
{
    utils::AutoMutex autoLock(dataLock_);
    
    processTaskQueue();
    calculateNewCommand();

    haveState_ = false;
    dataTrigger_.setPredicate(false);

    transmitter.sendMessage(currentCommand_);

    // motion_controller always runs
    return system::UpdateStatus::running;
}


void MotionControllerDirector::shutdown(system::ModuleCommunicator& transmitter)
{
    robot::motion_command_t zeroCommand(robot::AUTONOMOUS_CONTROLLER,
                                        robot::velocity_command_t(0.0f, 0.0f),
                                        robot::joystick_command_t(0, 0, 100));
    zeroCommand.timestamp = utils::system_time_us();
    
    transmitter.sendMessage(zeroCommand); // send zero cammand at shutdown
}


void MotionControllerDirector::handleData(const std::shared_ptr<MotionControllerTask>& task, const std::string& channel)
{
    // TODO: There are two places from which paths can arrive. The user and the robot. Need to
    //       decide exactly how they should interact. For now, just erase any prior path when
    //       a new path arrives
    utils::AutoMutex autoLock(dataLock_);
    
    // hadling cancle command
    if(isCancelled_)
    {
        std::cout<<"Received task after being cancelled: Cancel time:"<<cancelledTimestamp_<<" Task time:"<<task->getTimestamp()<<'\n';
    }

    if(isCancelled_ && (task->getTimestamp() < cancelledTimestamp_))
    {
        return;
    }
    else
    {
        if(isCancelled_)
        {
            std::cout<<"Resuming motion after previous task was cancelled.\n";
        }
        isCancelled_ = false;
    }
    
    // validation of the received task based on timestamp and queuing
    if(task->getTimestamp() == 0) // timestamp 0 only happens if the task was constructed with the default constructor.
    {
        std::cout<<"Warning: Zero timestamp on received MotionControllerTask! Ignoring command.\n\n";
    }
    else
    {
        taskQueue_.push_back(task);
    }
    
    // Don't wakeup when a new task arrives because we can only perform feedback with a new state
    // just wait until that state arrives for processing the new task
}


void MotionControllerDirector::handleData(const motion_state_t& state, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    currentState_ = state;
    haveState_    = true;

    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void MotionControllerDirector::handleData(const motion_controller_command_message_t& message, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    switch(message.command)
    {
    case MOTION_CONTROLLER_PAUSE:
        isPaused_        = true;
        pausedTimestamp_ = message.timestamp;
        
        controllers_.setToPause(utils::sec_to_usec(1)); //
        
        std::cout<<"Paused following: Time:"<<message.timestamp<<'\n';
        break;

    case MOTION_CONTROLLER_RESUME:
        isPaused_ = false;
        
        controllers_.resume();
        break;

    case MOTION_CONTROLLER_CANCEL:
        isCancelled_        = true;
        cancelledTimestamp_ = message.timestamp;
        
        std::cout<<"Cancelled following: Time:"<<message.timestamp<<'\n';
        break;
    }

    // Wakeup if a new message has arrived because the current task could have been cancelled, thus the 
    // wheelchair should be stopping immediately
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void MotionControllerDirector::processTaskQueue(void)
{
    // Go through in order, the most current task will be the one that will be executed
    for(const auto& task : taskQueue_)
    {
        controllers_.assignTask(task);
    }
    
    taskQueue_.clear();
}


void MotionControllerDirector::calculateNewCommand(void)
{
    int64_t currentTimeUs = utils::system_time_us();
    
    // if the descrepancy between the timestamp in the data and the current system time is large use timestamp in the data.
    if(std::abs(currentTimeUs - currentState_.timestamp) > 100000)
    {
        currentTimeUs = currentState_.timestamp;
    }
    
    if(shouldCalculateCommand())
    {
        motion_controller_data_t data;
        data.state         = currentState_;
        data.currentTimeUs = currentTimeUs;
        data.timestep      = 0.05f;
        
        currentCommand_ = controllers_.calculateCommand(data); // source and timestamp updated within the ControllerChain    
    }
    else
    {
        // send zero command if the command is not to be calculated
        currentCommand_ = robot::motion_command_t(robot::AUTONOMOUS_CONTROLLER,
                                                  robot::velocity_command_t(0.0f, 0.0f),
                                                  robot::joystick_command_t(0, 0, 100));
        currentCommand_.timestamp = utils::system_time_us();
    }
}


bool MotionControllerDirector::shouldCalculateCommand(void)
{
    return haveState_ && !isCancelled_;
}

} // namespace planner
} // namespace vulcan
