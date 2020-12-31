/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     messages.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of message types for the motion controller :
 *
 *  - command to pause, resume or cancel the controller
 */

#ifndef MPEPC_MOTION_CONTROLLER_MESSAGES_H
#define MPEPC_MOTION_CONTROLLER_MESSAGES_H

#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cstdint>
#include <string>

namespace vulcan
{
namespace mpepc
{

/**
 * motion_controller_command_message_t allows for control of the behavior of the motion controller.
 * The motion controller action can be paused, resumed, or outright cancelled.
 */
enum motion_controller_command_t
{
    MOTION_CONTROLLER_PAUSE,
    MOTION_CONTROLLER_RESUME,
    MOTION_CONTROLLER_CANCEL
};

struct motion_controller_command_message_t
{
    int64_t timestamp;
    motion_controller_command_t command;
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, motion_controller_command_message_t& message)
{
    ar(message.timestamp, message.command);
}

}   // namespace mpepc
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(mpepc::motion_controller_command_message_t, ("MOTION_CONTROLLER_COMMAND_MESSAGE"))

#endif   // MPEPC_MOTION_CONTROLLER_MESSAGES_H
