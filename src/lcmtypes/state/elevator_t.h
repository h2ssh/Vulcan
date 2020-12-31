/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_STATE_ELEVATOR_T_H
#define LCMTYPES_STATE_ELEVATOR_T_H

#include "lcmtypes/vulcan_lcm_elevator_t.h"
#include <string>

namespace vulcan
{
namespace robot
{
struct elevator_t;
}

namespace lcm
{

const std::string ELEVATOR_CHANNEL("STATE_ELEVATOR");

void convert_lcm_to_vulcan(const vulcan_lcm_elevator_t& elevatorMessage, robot::elevator_t& elevator);
void convert_vulcan_to_lcm(const robot::elevator_t& elevator, vulcan_lcm_elevator_t& elevatorMessage);

void publish_data(lcm_t* lcm, const robot::elevator_t& elevator, std::string channel = ELEVATOR_CHANNEL);
void subscribe_to_message(lcm_t* lcm,
                          void (*callback)(const robot::elevator_t&, const std::string&, void*),
                          void* userdata,
                          std::string channel = ELEVATOR_CHANNEL);

}   // namespace lcm
}   // namespace vulcan

#endif   // LCMTYPES_STATE_ELEVATOR_T_H
