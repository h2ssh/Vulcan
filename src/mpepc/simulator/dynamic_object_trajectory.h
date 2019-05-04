/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_trajectory.h
* \author   Jong Jin Park and Collin Johnson
*
* Definition of DynamicObjectType, dynamic_object_state_t, and dynamic_object_trajectory_t.
*/

#ifndef MPEPC_DYNAMIC_OBJECT_TRAJECTORY_H
#define MPEPC_DYNAMIC_OBJECT_TRAJECTORY_H

#include <core/pose.h>
#include <tracker/dynamic_object.h>
#include <tracker/object_state.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{

namespace mpepc
{

/**
* DynamicObjectType is an identifier for the type of object represented by a particular trajectory. It's use is really
* just for visualization purposes at the moment.
*/
enum class DynamicObjectType
{
    pedestrian,
    invisible,
    other,
};

/**
* dynamic_object_state_t stores state of objects common to all object types
*/
using dynamic_object_state_t = tracker::object_motion_state_t;


/**
* dynamic_object_trajectory_t stores object type, timestamp and the series of states along the trajectory.
*/
struct dynamic_object_trajectory_t
{
    DynamicObjectType type; // type of the object.

    int64_t timestamp; // timestamp at the beginning of the trajectory.
    float   timestep;  // timestep between simulated states.
    double  uncertaintyStd; // positional uncertainty std (circular approximation)

    double priorProbability; // uncertainty with the object that trajectory was generated from

    pose_t goal;                 // goal for the object
    Point<float> preferredVel;    // vector (xVel, yVel) preferred velocity for the object once it reaches the goal

    std::vector<dynamic_object_state_t> states;
    double radius;

    tracker::DynamicObject::Ptr laserObject; // original data from which the object trajectory were estimated. Rich set of data exists in here.
};


struct dynamic_object_trajectory_debug_info_t
{
    DynamicObjectType type; // type of the object

    int64_t timestamp; // timestamp at the beginning of the trajectory.
    float   timestep;  // timestep between simulated states.

    std::vector<dynamic_object_state_t> states; // *sub-sampled* staets of the simulated object.

    pose_t goal;                 // goal for the object
    Point<float> preferredVel;    // preferred velocity for the object once it reaches the goal

    double priorProbability; // probability of existence for the object that trajectory was generated from
    double radius;      // estimated radius of the object, which is used for collision detection

    dynamic_object_trajectory_debug_info_t(void) = default;

    explicit dynamic_object_trajectory_debug_info_t(const dynamic_object_trajectory_t& trajectory,
                                                    std::size_t subsampleRate = 4)
    : type(trajectory.type)
    , timestamp(trajectory.timestamp)
    , timestep(trajectory.timestep)
    , goal(trajectory.goal)
    , preferredVel(trajectory.preferredVel)
    , radius(trajectory.radius)
    {
        // subsampling
        for(size_t index = 0; index < trajectory.states.size(); index += subsampleRate)
        {
            states.push_back(trajectory.states[index]);
        }
    }
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, dynamic_object_state_t& state, const unsigned int version)
{
    ar (state.x,
        state.y,
        state.xVel,
        state.yVel,
        state.xAccel,
        state.yAccel);
}

template <class Archive>
void serialize(Archive& ar, dynamic_object_trajectory_t& trajectory, const unsigned int version)
{
    ar (trajectory.type,
        trajectory.timestamp,
        trajectory.timestep,
        trajectory.states,
        trajectory.radius,
        trajectory.uncertaintyStd,
        trajectory.priorProbability,
        trajectory.goal,
        trajectory.preferredVel,
        trajectory.laserObject);
}


template <class Archive>
void serialize(Archive& ar, dynamic_object_trajectory_debug_info_t& trajectory, const unsigned int version)
{
    ar (trajectory.type,
        trajectory.timestamp,
        trajectory.timestep,
        trajectory.states,
        trajectory.priorProbability,
        trajectory.goal,
        trajectory.preferredVel,
        trajectory.radius);
}

} // mpepc
} // vulcan

DEFINE_DEBUG_MESSAGE(std::vector<mpepc::dynamic_object_trajectory_t>, ("MPEPC_DYNAMIC_OBJECT_TRAJECTORIES"))
DEFINE_DEBUG_MESSAGE(std::vector<mpepc::dynamic_object_trajectory_debug_info_t>, ("MPEPC_DYNAMIC_OBJECT_TRAJECTORIES_DEBUG_INFO"))

#endif // MPEPC_DYNAMIC_OBJECT_TRAJECTORY_H
