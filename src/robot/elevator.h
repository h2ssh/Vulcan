/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     elevator.h
 * \author   Collin Johnson
 *
 * Declaration of types associated with the status of detected elevator motion:
 *
 *   - elevator_state_t  : encodes status of elevator motion up/down/stopped
 *   - elevator_t        : state of elevator plus estimation of vertical motion of current elevator trip
 */

#ifndef STATE_ELEVATOR_H
#define STATE_ELEVATOR_H

#include "system/message_traits.h"
#include <cstdint>

namespace vulcan
{
namespace robot
{

/**
 * elevator_state_t encodes the possible states of motion for travel on an elevator.
 * An elevator is either stopped or moving up or down.
 */
enum elevator_state_t
{
    ELEVATOR_MOVING_UP,
    ELEVATOR_MOVING_DOWN,
    ELEVATOR_STOPPED
};

/**
 * elevator_t defines the state of elevator motion as detected by the state_monitor.
 */
struct elevator_t
{
    int64_t timestamp;

    elevator_state_t state;   ///< State in which the elevator is in

    int64_t startTime;   ///< Time the elevator started moving
    int64_t stopTime;    ///< Time the elevator stopped moving

    double distance;   ///< Vertical distance the elevator has travelled since motion began
    double velocity;   ///< Current vertical velocity of the robot
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, elevator_t& e)
{
    ar(e.timestamp, e.state, e.startTime, e.stopTime, e.distance, e.velocity);
}

}   // namespace robot
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(robot::elevator_t, ("ROBOT_ELEVATOR_STATE"))

#endif   // STATE_ELEVATOR_H
