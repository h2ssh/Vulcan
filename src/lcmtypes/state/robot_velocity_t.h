/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_ROBOT_VELOCITY_T_H
#define LCMTYPES_ROBOT_VELOCITY_T_H

#include <lcmtypes/vulcan_lcm_robot_velocity_t.h>
#include <string>

namespace vulcan
{
struct velocity_t;

namespace lcm
{

const std::string VELOCITY_CHANNEL("STATE_VELOCITY");

void convert_lcm_to_vulcan(const vulcan_lcm_robot_velocity_t& velocityMessage, velocity_t& velocity);
void convert_vulcan_to_lcm(const velocity_t& velocity, vulcan_lcm_robot_velocity_t& velocityMessage);

void publish_data(lcm_t* lcm, const velocity_t& velocity, std::string channel = VELOCITY_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const velocity_t&, const std::string&, void*), void* userdata, std::string channel = VELOCITY_CHANNEL);

}
}

#endif // LCMTYPES_ROBOT_VELOCITY_T_H
