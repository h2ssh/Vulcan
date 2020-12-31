/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     waypoint_decider.cpp
* \author   Collin Johnson
*
* Definition of WaypointTargetDecider. LOG_DATA flag optionally saves the commanded paths
* to path.log.
*/

#include "mpepc/motion_controller/waypoint_follower/waypoint_decider.h"
#include "core/point.h"
#include "core/angle_functions.h"

namespace vulcan
{

namespace mpepc
{
    
WaypointDecider::WaypointDecider(void)
{
}


void WaypointDecider::setPath(const controller_waypoint_path_t& path)
{
    this->path          = path;
    targetWaypointIndex = 0;
}


void WaypointDecider::clearPath(void)
{
    this->path.waypoints.clear();
    targetWaypointIndex = 0;
}


bool WaypointDecider::shouldContinueDriving(const pose_t& currentPose) const
{
    return !haveReachedGoal();
}


bool WaypointDecider::haveReachedTargetWaypoint(const pose_t& currentPose) const
{
    if(haveReachedGoal())
    {
        return true;
    }

    controller_waypoint_t waypoint = getTargetWaypoint();

    float distanceToWaypoint = distance_between_points(Point<float>(currentPose.x, currentPose.y),
                                                             Point<float>(waypoint.pose.x, waypoint.pose.y));

    float waypointAlignment = fabs(angle_diff(currentPose.theta, waypoint.pose.theta));

    if(targetWaypointIndex == path.waypoints.size()-1)
    {
        return (distanceToWaypoint < waypoint.radius/2) && (waypointAlignment < waypoint.alignment);
    }
    else
    {
        return (distanceToWaypoint < waypoint.radius) && (waypointAlignment < waypoint.alignment);
    }
}


uint16_t WaypointDecider::selectNextTargetWaypoint(const pose_t& currentPose)
{
    // No need to look at the pose for now because the waypoints are knocked off sequentially. If the robot
    // is close enough to two waypoints because they are very close, then successive updates of the
    // controller will result in reached waypoints

    return (targetWaypointIndex < path.waypoints.size()) ? ++targetWaypointIndex : targetWaypointIndex;
}


controller_waypoint_t WaypointDecider::getTargetWaypoint(void) const
{
    controller_waypoint_t nextWaypoint;

    if(!haveReachedGoal())
    {
        nextWaypoint = path.waypoints[targetWaypointIndex];
    }
    else
    {
        nextWaypoint.pose            = pose_t(0, 0, 0);
        nextWaypoint.radius          = 0;
        nextWaypoint.attenuationTime = 0;
    }

    return nextWaypoint;
}


bool WaypointDecider::haveReachedGoal(void) const
{
    return targetWaypointIndex >= path.waypoints.size();
}

} // namespace planner
} // namespace vulcan
