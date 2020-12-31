/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_controller.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of MotionController interface and create_motion_controller factory.
 */

#ifndef MPEPC_MOTION_CONTROLLER_H
#define MPEPC_MOTION_CONTROLLER_H

#include <memory>
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
class MotionController;
struct motion_controller_params_t;
struct motion_controller_data_t;

/**
 * create_motion_controller is a factory function that creates a new instance of MotionController based
 * on the provided type string description of the controller.
 *
 * If type is not a known MotionController, the program will crash.
 *
 * \pre      Type is a valid description of a subclass of MotionController
 * \param    type            String identifier for the type to be created
 * \param    params          Parameters for the controller
 * \return   A new instance of the desired subclass of MotionController.
 */
std::unique_ptr<MotionController> create_motion_controller(const std::string& type,
                                                           const motion_controller_params_t& params);

/**
 * MotionController is an interface
 */
class MotionController
{
public:
    /**
     * canHandleTask checks to see if this MotionController can handle the desired control task.
     *
     * \param    task            Task to be performed
     * \return   True if the task can be handled. False otherwise.
     */
    virtual bool canHandleTask(const std::shared_ptr<MotionControllerTask>& task) = 0;

    /**
     * assignTask provides a new task for the MotionController to process. On subsequent
     * calls, updateCommand will be called until a new task arrives.
     *
     * \pre      canHandleTask(task) == true
     * \param    task            Task being assigned
     */
    virtual void assignTask(const std::shared_ptr<MotionControllerTask>& task) = 0;

    /**
     * isTaskComplete checks to see if the current controller task has been completed.
     *
     * \param    data            Current robot state
     * \return   True if the task has completed. False if work remains.
     */
    virtual bool isTaskComplete(const motion_controller_data_t& data) = 0;

    /**
     * updateCommand will update the current controller command with new state information from the robot.
     *
     * \param    state           Current robot and controller state to use for the update
     * \return   A new motion command to assign to the robot.
     */
    virtual robot::motion_command_t updateCommand(const motion_controller_data_t& data) = 0;

    /**
     * pauseCommand will pause the current controller command over the specified interval.
     *
     * \param    timeUs          Time over which the controller will stop to a halt.
     */
    virtual void pauseCommand(int64_t timeUs) = 0;

    /**
     * resumeCommand will resume the current command.
     */
    virtual void resumeCommand(void) = 0;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_MOTION_CONTROLLER_H
