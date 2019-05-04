/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_COMMANDED_JOYSTICK_T_H
#define LCMTYPES_COMMANDED_JOYSTICK_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_commanded_joystick_t.h>

namespace vulcan
{
namespace robot { struct commanded_joystick_t; }

namespace lcm
{
    
const std::string COMMANDED_JOYSTICK_CHANNEL("SENSOR_COMMANDED_JOYSTICK");

void convert_lcm_to_vulcan(const vulcan_lcm_commanded_joystick_t& joystickMessage, vulcan::robot::commanded_joystick_t& joystick);
void convert_vulcan_to_lcm(const vulcan::robot::commanded_joystick_t& joystick, vulcan_lcm_commanded_joystick_t& joystickMessage);

void publish_data(lcm_t* lcm, const vulcan::robot::commanded_joystick_t& joystick, std::string channel = COMMANDED_JOYSTICK_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const vulcan::robot::commanded_joystick_t&, const std::string&, void*), void* userdata, std::string channel = COMMANDED_JOYSTICK_CHANNEL);

}
}

#endif // LCMTYPES_COMMANDED_JOYSTICK_T_H
