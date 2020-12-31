/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     controller_chain.cpp
* \author   Collin Johnson
* 
* Definition of ControllerChain.
*/

#include "mpepc/motion_controller/controller/controller_chain.h"
#include "mpepc/motion_controller/controller/motion_controller.h"
#include "mpepc/motion_controller/task/task.h"
#include "mpepc/motion_controller/data.h"
#include "robot/commands.h"
#include "utils/timestamp.h"
#include <iostream>

namespace vulcan
{

namespace mpepc
{
    
const int INVALID_CONTROLLER_INDEX = -1;

ControllerChain::ControllerChain(const std::vector<std::string>& controllerTypes, const motion_controller_params_t& params)
    : activeControllerIndex(INVALID_CONTROLLER_INDEX)
{
    for(const auto& type : controllerTypes)
    {
        controllers.push_back(create_motion_controller(type, params));
    }
}


bool ControllerChain::assignTask(const std::shared_ptr<MotionControllerTask>& task)
{
    for(size_t n = 0; n < controllers.size(); ++n)
    {
        if(controllers[n]->canHandleTask(task))
        {
            controllers[n]->assignTask(task);
            activeControllerIndex = n;
            return true;
        }
    }
    
    std::cerr<<"ERROR:ControllerChain: No controller is able to handle task: "<<task->getDescription()<<'\n';
    
    activeControllerIndex = INVALID_CONTROLLER_INDEX;
    
    return false;
}


robot::motion_command_t ControllerChain::calculateCommand(const motion_controller_data_t& data)
{
    if(activeControllerIndex == INVALID_CONTROLLER_INDEX)
    {
        // set zero command and manually populate fields
        robot::motion_command_t stopCommand(robot::AUTONOMOUS_CONTROLLER,
                                            robot::velocity_command_t(0.0f, 0.0f),
                                            robot::joystick_command_t(0, 0, 100));
        stopCommand.timestamp   = utils::system_time_us(); // always carry timestamp at data genertion
        
        return stopCommand;
    }
    
    return controllers[activeControllerIndex]->updateCommand(data);
}


void ControllerChain::setToPause(int64_t timeUs)
{
    if(activeControllerIndex != INVALID_CONTROLLER_INDEX)
    {
        controllers[activeControllerIndex]->pauseCommand(timeUs);
    }
}


void ControllerChain::resume(void)
{
    if(activeControllerIndex != INVALID_CONTROLLER_INDEX)
    {
        controllers[activeControllerIndex]->resumeCommand();
    }
}


} // mpepc
} // vulcan
