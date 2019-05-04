/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_JOYSTICK_COMMAND_T_H
#define LCMTYPES_JOYSTICK_COMMAND_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_joystick_command_t.h>

namespace vulcan
{
namespace robot { struct joystick_command_t; }

namespace lcm
{
    
const std::string JOYSTICK_COMMAND_CHANNEL("ROBOT_JOYSTICK_COMMAND");

void convert_lcm_to_vulcan(const vulcan_lcm_joystick_command_t& joystickMessage, robot::joystick_command_t& joystick);
void convert_vulcan_to_lcm(const robot::joystick_command_t& joystick, vulcan_lcm_joystick_command_t& joystickMessage);

void publish_data(lcm_t* lcm, const robot::joystick_command_t& joystick, std::string channel = JOYSTICK_COMMAND_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const robot::joystick_command_t&, const std::string&, void*), void* userdata, std::string channel = JOYSTICK_COMMAND_CHANNEL);

}
}

#endif // LCMTYPES_JOYSTICK_COMMAND_T_H
