/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_VELOCITY_COMMAND_T_H
#define LCMTYPES_VELOCITY_COMMAND_T_H

#include "lcmtypes/vulcan_lcm_velocity_command_t.h"
#include <string>

namespace vulcan
{
namespace robot
{
struct velocity_command_t;
}

namespace lcm
{

const std::string VELOCITY_COMMAND_CHANNEL("ROBOT_VELOCITY_COMMAND");

void convert_lcm_to_vulcan(const vulcan_lcm_velocity_command_t& velocityMessage, robot::velocity_command_t& velocity);
void convert_vulcan_to_lcm(const robot::velocity_command_t& velocity, vulcan_lcm_velocity_command_t& velocityMessage);

void publish_data(lcm_t* lcm,
                  const robot::velocity_command_t& velocity,
                  const std::string& channel = VELOCITY_COMMAND_CHANNEL);
void subscribe_to_message(lcm_t* lcm,
                          void (*callback)(const robot::velocity_command_t&, const std::string&, void*),
                          void* userdata,
                          std::string channel = VELOCITY_COMMAND_CHANNEL);

}   // namespace lcm
}   // namespace vulcan

#endif   // LCMTYPES_VELOCITY_COMMAND_T_H
