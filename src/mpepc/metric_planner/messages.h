/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     messages.h
* \author   Jong Jin Park
*
* Declaration of message types for the local metric planner.
*
* The messages defined here are used to handle special cases, such as external inputs
* to pause, cancel or resume the planner, or detection of failures or other special conditions.
*/


#ifndef MPEPC_METRIC_PLANNER_MESSAGES_H
#define MPEPC_METRIC_PLANNER_MESSAGES_H

#include <mpepc/metric_planner/task/task.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cstdint>

namespace vulcan
{

namespace mpepc
{

/**
* metric_planner_command_t and metric_planner_command_message_t is used to pause, resume, or cancel the planning process.
*/
enum metric_planner_command_t
{
    PAUSE,                           // pause the current trajectory planning process, run path planner if applicable, keep all data, stop the robot.
    RESUME,                          // resume all planning process
    CANCEL,                          // stop the all planning process, clear all data, stop the robot.
    UPDATE_WAYPOINTS,                // update waypoints (using RRT star if applicable)
    SCRIPT_NEXT_TASK,                // allow to skip a task in a script
    SCRIPT_STOP,                     // allow stop/cancelling the script
    SCRIPT_LOOP,                     // allow looping through the script
    SET_MODE_STOP_AT_WAYPOINTS,      // robot stops at waypoints
    SET_MODE_USE_RRT_STAR,           // planner uses RRT star to validate and fill-in the waypoints
    SET_MODE_QUASI_STATIC_OBJECTS,   // ignore object velocities (quasi-static world assumption)
    SET_MODE_SIMPLE_POSE_FOLLOWING,  // activate simple pose following mode, where the optimizer only optimizes the velocity gain. The pose of the motion target is fixed at the next waypoint.
};

struct metric_planner_command_message_t
{
    int64_t                  timestamp; // useful when implementing timeout
    metric_planner_command_t command;   // command type
    int                      intValue;  // int value holding relevant information

    metric_planner_command_message_t(void)
    : timestamp(0)
    , command(CANCEL)
    , intValue(0)
    {
    }
};


/**
* metric_planner_status_t and metric_planner_status_message_t is used to report the status of the planner (e.g. failure/success).
* useful for display purposes.
*/
enum metric_planner_status_t
{
    // not running
    IDLE,   // has no task to perform
    PAUSED, // has a task but is paused

    // running
    ACTIVE_NORMAL,  // normal trajectory planning
    ACTIVE_SPECIAL, // any special mode
    ACTIVE_RRT,     // running rrt only

    // success
    SUCCESS_REACHED_POSE, // reached the target pose at this cycle.

    // failure
    FAILURE_CANNOT_ASSIGN_TASK,  // cannot assign task
    FAILURE_UNABLE_TO_PROGRESS,  // cannot progress for unexpectedly long duration
    FAILURE_CANNOT_FIND_SOLUTION // numerical solver cannot find a valid solution

//     ACTIVE, // is actively performing a task
//
//     // failure meessages
//     CANNOT_ASSIGN_TASK,   // task has failed to assign
//     UNABLE_TO_PROGRESS,   // cannot progress toward completing the task
//     CANNOT_FIND_SOLUTION, // the planner has failed to find a solution
//
//     // success messages
//     REACHED_DESTINATION_POSE // finished a navigation task
};

struct metric_planner_status_message_t
{
    int64_t                 timestamp;      ///< Timestamp the status message was created
    metric_planner_status_t status;         ///< Status of the planner
    MetricPlannerTask::Id   taskId;         ///< Id of the currently running task, if running a task
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, metric_planner_command_message_t& message)
{
    ar (message.timestamp,
        message.command,
        message.intValue);
}

template <class Archive>
void serialize(Archive& ar, metric_planner_status_message_t& message)
{
    ar (message.timestamp,
        message.status,
        message.taskId);
}

} // mpepc
} // vulcan

DEFINE_SYSTEM_MESSAGE(mpepc::metric_planner_command_message_t, ("MPEPC_METRIC_PLANNER_COMMAND_MESSAGE"))
DEFINE_DEBUG_MESSAGE(mpepc::metric_planner_status_message_t, ("MPEPC_METRIC_PLANNER_STATUS_MESSAGE")) // this will needs to change to a system message once there are other modules using this status

#endif // MPEPC_METRIC_PLANNER_MESSAGES_H
