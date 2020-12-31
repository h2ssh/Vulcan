/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_following_controller.cpp
 * \author   Collin Johnson
 *
 * Definition of PathFollowingController.
 */

#include "mpepc/motion_controller/controller/path_following_controller.h"
#include "mpepc/motion_controller/data.h"
#include "robot/commands.h"
#include <cassert>
#include <iostream>

// #define DEBUG_COMMAND
// #define DEBUG_PATH

namespace vulcan
{

namespace mpepc
{

PathFollowingController::PathFollowingController(const waypoint_follower_params_t& params)
: reachedWaypoint(false)
, updateCount(0)
, follower(params)
{
}


bool PathFollowingController::canHandleTask(const std::shared_ptr<MotionControllerTask>& task)
{
    return task->getId() == PathFollowingTask::PATH_FOLLOWING_TASK_ID;
}


void PathFollowingController::assignTask(const std::shared_ptr<MotionControllerTask>& task)
{
    assert(task->getId() == PathFollowingTask::PATH_FOLLOWING_TASK_ID);

    std::shared_ptr<PathFollowingTask> pathTask = std::static_pointer_cast<PathFollowingTask>(task);

    currentPath = pathTask->getPath();

    decider.setPath(currentPath);
    follower.setWaypointTarget(decider.getTargetWaypoint());
}


bool PathFollowingController::isTaskComplete(const motion_controller_data_t& data)
{
    return !decider.shouldContinueDriving(data.state.pose);
}


robot::motion_command_t PathFollowingController::updateCommand(const motion_controller_data_t& data)
{
    robot::motion_command_t newCommand;

    if (!decider.shouldContinueDriving(data.state.pose)) {
        newCommand.velocityCommand = robot::velocity_command_t(0.0f, 0.0f, data.state.timestamp);
    } else {
        newCommand.velocityCommand = follower.calculateVelocityCommand(data.state, data.timestep);
        newCommand.velocityCommand.timestamp = data.state.timestamp;
    }

    newCommand.commandType = robot::VELOCITY;
    newCommand.source = robot::AUTONOMOUS_CONTROLLER;
    newCommand.timestamp = data.currentTimeUs;


#ifdef DEBUG_COMMAND
    if ((newCommand.velocityCommand.linearVelocity != 0.0f || newCommand.velocityCommand.angularVelocity != 0.0f)
        && (++updateCount % 10 == 0)) {
        std::cout << "INFO: PathFollowingController: command:(" << newCommand.linearVelocity << ','
                  << newCommand.angularVelocity << ")\n";
    }
#endif

    return newCommand;
}


void PathFollowingController::determineTargetWaypoint(const pose_t& pose)
{
    reachedWaypoint = decider.haveReachedTargetWaypoint(pose);

    if (reachedWaypoint) {
#ifdef DEBUG_PATH
        std::cout << "INFO: PathFollowingController: Reached waypoint: " << decider.getTargetWaypoint().pose << '\n';
#endif

        previousWaypointIndex = decider.selectNextTargetWaypoint(pose) - 1;

        if (decider.shouldContinueDriving(pose)) {
            follower.setWaypointTarget(decider.getTargetWaypoint());
        } else {
            follower.pathCompleted();
        }
    }
}


void PathFollowingController::pauseCommand(int64_t timeUs)
{
    // TODO
}


void PathFollowingController::resumeCommand(void)
{
    // TODO
}


}   // namespace mpepc
}   // namespace vulcan
