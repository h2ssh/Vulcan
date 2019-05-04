/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path.h
* \author   Collin Johnson
* 
* Declaration of controller_waypoint_t, controller_waypoint_path_t, and PathFollowingTask.
*/

#ifndef MPEPC_MOTION_CONTROLLER_PATH_FOLLOWING_TASK_H
#define MPEPC_MOTION_CONTROLLER_PATH_FOLLOWING_TASK_H

#include <mpepc/motion_controller/task/task.h>
#include <core/pose.h>
#include <core/angle_functions.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace mpepc
{

/**
* controller_waypoint_t represents a node in a path through the LPM. Each node is represented by:
*
*   - pose                  : target pose for the robot to attain for this waypoint in the current LPM reference frame
*   - radius                : maximum distance from the waypoint for it to be cleared by the robot
*   - alignment             : maximum difference in orientation alignment for the waypoint to be cleared
*   - attenuationTime       : amount of time to attentuate the control signal when shifting to this waypoint from a prior target
*   - useWaypointVelocity   : flag indicating if this waypoint is a fixed velocity waypoint, where the provided linear velocity should be used
*   - velocity              : velocity gain to use for driving to the waypoint
*/
struct controller_waypoint_t
{
    pose_t pose;
    float         radius;
    float         alignment;
    int64_t       attenuationTime;
    
    bool  useWaypointVelocity;     // Flag indicating this target should be approached at a fixed linear velocity, as specified
    float velocity;
    
    /**
    * reachedWaypoint checks to see if the robot has reached the waypoint based on the current pose. A waypoint has
    * been reached if the current pose is within radius distance of the waypoint and the orientation alignment is
    * within the alignment bound.
    *
    * \param    currentPose             Current pose of the robot
    * \return   True if the currentPose is close enough to the waypoint as specified by radius and alignment.
    */
    bool reachedWaypoint(const pose_t& currentPose)
    {
        return (distance_between_points(pose.toPoint(), currentPose.toPoint()) < radius) &&
               (fabs(angle_diff(currentPose.theta, pose.theta)) < alignment);
    }
    
    controller_waypoint_t(void)
        : useWaypointVelocity(false)
        , velocity(0.0f)
    {
    }
};

/**
* controller_waypoint_path_t is an ordered list of waypoints for the robot to follow. Each waypoint specifies
* a target in current global LPM frame of reference. The pose of the robot when the path was determined is
* also a part of the representation because the controller may need this information in order to maintain
* smooth control of the robot, depending on the stability of the localization estimate.
*/
struct controller_waypoint_path_t
{
    int64_t       timestamp;
    pose_t referencePose;
    
    std::vector<controller_waypoint_t> waypoints;
};

/**
* PathFollowingTask tells the motion controller that the robot should follow a series of waypoints.
*/
class PathFollowingTask : public MotionControllerTask
{
public:
    
    static const int32_t PATH_FOLLOWING_TASK_ID = 2;
    
    /**
    * Constructor for PathFollowingTask.
    * 
    * \param    path            Path associated with the task
    * \param    timestamp       Timestamp to assign to the task (optional, default = current time)
    */
    PathFollowingTask(const controller_waypoint_path_t& path, int64_t timestamp = 0)
    : MotionControllerTask(timestamp)
    , path(path)
    {
    }
    
    /**
    * getPath retrieves the path associated with the task.
    */
    controller_waypoint_path_t getPath(void) const { return path; }
    
    // MotionTargetTask interface
    virtual int32_t     getId(void)          const { return PATH_FOLLOWING_TASK_ID; }
    virtual std::string getDescription(void) const { return std::string("path following"); }
    
protected:
    
    // For serialization support, allow subclasses to default construct
    PathFollowingTask(void) { }
    
private:
    
    controller_waypoint_path_t path;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar & ::cereal::base_class<MotionControllerTask>(this);
        ar & path;
    }
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, controller_waypoint_t& waypoint)
{
    ar (waypoint.pose,
        waypoint.radius,
        waypoint.alignment,
        waypoint.attenuationTime,
        waypoint.useWaypointVelocity,
        waypoint.velocity);
}

template <class Archive>
void serialize(Archive& ar, controller_waypoint_path_t& waypoint)
{
    ar (waypoint.timestamp,
        waypoint.referencePose,
        waypoint.waypoints);
}

} // mpepc
} // vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::PathFollowingTask)

#endif // MPEPC_MOTION_CONTROLLER_PATH_FOLLOWING_TASK_H
