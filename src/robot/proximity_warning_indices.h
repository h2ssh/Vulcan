/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     proximity_warning_indices.h
 * \author   Collin Johnson
 *
 * Declaration of proximity_warning_indices_t type.
 */

#ifndef ROBOT_PROXIMITY_WARNING_INDICES_H
#define ROBOT_PROXIMITY_WARNING_INDICES_H

#include "system/message_traits.h"
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace robot
{

/**
 * proximity_warning_indices_t contains information about laser scan points that violated the proximity
 * bounds of the low-level collision avoidance system. The collision avoidance system simply halts the
 * robot when objects are too close.
 *
 * The indices and the scan number in which they were found is included. There are two types of proximity
 * violations: warnings and critical. Warning points mean something is potentially moving into a position where
 * it will collide with the robot. Critical points are on course for imminent, if not current, collision
 * with the robot.
 */
struct proximity_warning_indices_t
{
    int64_t timestamp;
    int32_t scanId;
    int32_t laserId;

    std::vector<unsigned int> warningIndices;
    std::vector<unsigned int> criticalIndices;
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, proximity_warning_indices_t& indices)
{
    ar(indices.timestamp, indices.scanId, indices.laserId, indices.warningIndices, indices.criticalIndices);
}

}   // namespace robot
}   // namespace vulcan

DEFINE_DEBUG_MESSAGE(robot::proximity_warning_indices_t, ("DEBUG_ROBOT_PROXIMITY_INDICES"))

#endif   // ROBOT_PROXIMITY_WARNING_INDICES_H
