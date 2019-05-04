/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_target_following_controller.cpp
* \author   Jong Jin Park and Collin Johnson
*
* Definition of MotionTargetFollowingController.
*/

#include <mpepc/motion_controller/controller/motion_target_following_controller.h>
#include <mpepc/motion_controller/data.h>

namespace vulcan
{

namespace mpepc
{

MotionTargetFollowingController::MotionTargetFollowingController(const motion_target_following_controller_params_t& params)
: kinematicControlLaw_(params.kinematicControlLawParams)
, joystickControlLaw_(params.joystickControlLawParams, params.robotParams)
, haveTarget_(false)
, params_(params)
{
}

bool MotionTargetFollowingController::canHandleTask(const std::shared_ptr<MotionControllerTask>& task)
{
    return task->getId() == MotionTargetTask::MOTION_TARGET_TASK_ID;
}


void MotionTargetFollowingController::assignTask(const std::shared_ptr<MotionControllerTask>& task)
{
    assert(task->getId() == MotionTargetTask::MOTION_TARGET_TASK_ID);

    std::shared_ptr<MotionTargetTask> motionTask = std::static_pointer_cast<MotionTargetTask>(task);

    target_          = motionTask->getTarget();
    targetTimestamp_ = motionTask->getTimestamp();
    targetTimeout_   = motionTask->getTimeout();

    isPaused_ = false;

    haveTarget_ = true;
}


bool MotionTargetFollowingController::isTaskComplete(const motion_controller_data_t& data)
{
    return kinematicControlLaw_.haveReachedTarget(data.state.pose, target_.pose, data.currentTimeUs);
}


robot::motion_command_t MotionTargetFollowingController::updateCommand(const motion_controller_data_t& data)
{
    assert(haveTarget_);

    if(data.currentTimeUs - targetTimestamp_ > targetTimeout_)
    {
        // set zero command if the task has timed out
        robot::motion_command_t stopCommand(robot::AUTONOMOUS_CONTROLLER,
                                            robot::velocity_command_t(0.0f, 0.0f),
                                            robot::joystick_command_t(0, 0, 100));
        stopCommand.timestamp = utils::system_time_us(); // always carry timestamp at data genertion

        std::cout<<"Current   timestamp : "<< data.currentTimeUs <<"\n";
        std::cout<<"Task      timestamp : "<< targetTimestamp_ <<"\n";
        std::cout<<"Specified timeout   : "<< targetTimeout_ <<"\n";
        std::cout<<"MotionTargetFollowingController: Task timed out. Sending stop command.\n";

        return stopCommand;
    }

    motion_target_t motionTarget = target_;

    if(isPaused_)
    {
        if(utils::system_time_us() < pauseEndTimeUs_)
        {
            motionTarget.velocityGain *= 1.0 - static_cast<double>(utils::system_time_us() - pauseStartTimeUs_) / static_cast<double>(pauseEndTimeUs_ - pauseStartTimeUs_);
        }
        else if(utils::system_time_us() > pauseEndTimeUs_)
        {
            motionTarget.velocityGain = 0.0;
        }
        else
        {
            motionTarget.velocityGain = 0.0;
            std::cout<<"Motion Target Controller: Pause error: Timestamp seems to be flowing backward!!\n";
        }
    }

    control_law_output_t controlLawOutput = kinematicControlLaw_.computeOutput(data.state.pose, motionTarget, data.currentTimeUs);

    robot::velocity_command_t velocityCommand(controlLawOutput.linearVelocity, controlLawOutput.angularVelocity);
    robot::joystick_command_t joystickCommand = joystickControlLaw_.computeOutput(data.state, controlLawOutput);

    robot::motion_command_t   motionCommand(robot::AUTONOMOUS_CONTROLLER, velocityCommand, joystickCommand);
    motionCommand.timestamp = data.currentTimeUs;

    return motionCommand;
}


void MotionTargetFollowingController::pauseCommand(int64_t timeUs)
{
    isPaused_ = true;
    pauseStartTimeUs_ = utils::system_time_us();
    pauseEndTimeUs_   = pauseStartTimeUs_ + timeUs;
}


void MotionTargetFollowingController::resumeCommand(void)
{
    isPaused_ = false;
}


} // namespace mpepc
} // namespace vulcan
