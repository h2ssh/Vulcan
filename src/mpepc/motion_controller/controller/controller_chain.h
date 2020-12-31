/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     controller_chain.h
 * \author   Collin Johnson
 *
 * Declaration of ControllerChain.
 */

#ifndef MPEPC_MOTION_CONTROLLER_CHAIN_H
#define MPEPC_MOTION_CONTROLLER_CHAIN_H

#include "mpepc/motion_controller/controller/motion_controller.h"
#include "mpepc/motion_controller/params.h"
#include <string>

namespace vulcan
{
namespace robot
{
struct motion_command_t;
}

namespace mpepc
{

class MotionControllerTask;
struct motion_controller_data_t;

/**
 * ControllerChain is a simple chain-of-responsibility implementation for handling MotionControllerTasks.
 * On construction, the chain has a simple sequence of possible MotionControllers capable of handling a task.
 * The order in which they are defined is the order in which the task is presented to them. The first controller
 * capable of handling a task will be assigned that task. Until a new task arrives, this controller will be the
 * one driving the robot.
 */
class ControllerChain
{
public:
    /**
     * Constructor for ControllerChain.
     *
     * \param    controllerTypes         Types of controllers to be loaded -- order is the order of creation
     * \param    params                  Parameters for the controllers
     */
    ControllerChain(const std::vector<std::string>& controllerTypes, const motion_controller_params_t& params);

    /**
     * assignTask assigns a new task to one of the controllers. If no controller can handle the task, then an
     * error will be printed and the controller won't do anything.
     *
     * \param    task                    New task for the controller
     * \return   True if the task was successfully assigned. False if no handler existed for the task.
     */
    bool assignTask(const std::shared_ptr<MotionControllerTask>& task);

    /**
     * calculateCommand calculates the motion command for the current task based on the robot's motion state.
     *
     * \param    data                    Data available for updating the command
     * \return   New velocity command to be issued.
     */
    robot::motion_command_t calculateCommand(const motion_controller_data_t& data);

    void setToPause(int64_t timeUs);
    void resume(void);

private:
    int activeControllerIndex;

    std::vector<std::unique_ptr<MotionController>> controllers;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_MOTION_CONTROLLER_CHAIN_H
