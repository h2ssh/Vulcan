/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_following_controller.h
 * \author   Collin Johnson
 *
 * Declaration of PathFollowingController.
 */

#ifndef MPEPC_PATH_FOLLOWING_CONTROLLER_H
#define MPEPC_PATH_FOLLOWING_CONTROLLER_H

#include "mpepc/motion_controller/controller/motion_controller.h"
#include "mpepc/motion_controller/task/path.h"
#include "mpepc/motion_controller/waypoint_follower/waypoint_decider.h"
#include "mpepc/motion_controller/waypoint_follower/waypoint_follower.h"

namespace vulcan
{

namespace mpepc
{

const std::string PATH_FOLLOWING_CONTROLLER_TYPE("path_following");

/**
 * PathFollowingController deals with PathFollowingTasks by moving smoothly along a series of
 * controller waypoints.
 */
class PathFollowingController : public MotionController
{
public:
    /**
     * Constructor for PathFollowingController.
     *
     * \param    params          Parameters for the waypoint follower
     */
    PathFollowingController(const waypoint_follower_params_t& params);

    /**
     * Destructor for PathFollowingController.
     */
    virtual ~PathFollowingController(void) { }

    // MotionController interface
    virtual bool canHandleTask(const std::shared_ptr<MotionControllerTask>& task);       /* override */
    virtual void assignTask(const std::shared_ptr<MotionControllerTask>& task);          /* override */
    virtual bool isTaskComplete(const motion_controller_data_t& data);                   /* override */
    virtual robot::motion_command_t updateCommand(const motion_controller_data_t& data); /* override */
    virtual void pauseCommand(int64_t timeUs);                                           /* override */
    virtual void resumeCommand(void);                                                    /* override */

private:
    void determineTargetWaypoint(const pose_t& pose);

    controller_waypoint_path_t currentPath;
    bool reachedWaypoint;
    uint16_t previousWaypointIndex;

    int updateCount;

    WaypointDecider decider;
    WaypointFollower follower;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_PATH_FOLLOWING_CONTROLLER_H
