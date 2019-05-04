/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     waypoint_decider.h
* \author   Collin Johnson
*
* Declaration of WaypointDecider for determining the current waypoint to be followed.
*/

#ifndef MPEPC_MOTION_CONTROLLER_WAYPOINT_TARGET_DECIDER_H
#define MPEPC_MOTION_CONTROLLER_WAYPOINT_TARGET_DECIDER_H

#include <mpepc/motion_controller/task/path.h>
#include <fstream>

namespace vulcan
{

namespace mpepc
{

/**
* WaypointDecider deicdes which waypoint the robot should be following based on the
* current path and current pose. Each path waypoint contains the radius and an alignment
* within which the robot needs to move in order to satisfy the goal condition. Once the
* pose is within the goal radius and alignment for a waypoint, then the next target is selected.
*
* The interface also supports asking if the goal pose has been reached. This condition is
* true when the robot has reached the final waypoint in the active path or if there is no
* active path.
*/
class WaypointDecider
{
public:

    /**
    * Constructor for WaypointDecider.
    */
    WaypointDecider(void);

    /**
    * setPath sets the path from which targets should be selected.
    */
    void setPath(const controller_waypoint_path_t& path);

    /**
    * clearPath erases the path currently being followered, thereby halting the robot.
    */
    void clearPath(void);

    /**
    * shouldContinueDriving checks to see if the robot continue moving along the currently assigned path
    * based on the current pose of the robot. When the robot has reached the end of its current path, this
    * method will return false.
    */
    bool shouldContinueDriving(const pose_t& currentPose) const;

    /**
    * haveReachedTargetWaypoint checks to see if the robot has made it to the current waypoint. If so,
    * then the next waypoint in the path needs to be determined by calling selectNextWaypoint().
    */
    bool haveReachedTargetWaypoint(const pose_t& currentPose) const;

    /**
    * selectNextTargetWaypoint determines the next waypoint for the robot to drive towards. The index of
    * the new waypoint is returned.
    */
    uint16_t selectNextTargetWaypoint(const pose_t& currentPose);

    /**
    * getTargetWaypoint retrieves the current waypoint target for the controller to follow. If there is
    * no target, i.e. haveReachedGoal() == true, then a waypoint of (0,0,0) is returned. This could
    * be a valid waypoint though! Summary: getTargetWaypoint() only returns a valid waypoint if
    * haveReachedGoal() == false.
    */
    controller_waypoint_t getTargetWaypoint(void) const;

private:

    bool haveReachedGoal(void) const;

    controller_waypoint_path_t path;
    uint16_t                   targetWaypointIndex;

    std::ofstream log;
};

} // mpepc
} // vulcan

#endif // MPEPC_MOTION_CONTROLLER_WAYPOINT_TARGET_DECIDER_H
