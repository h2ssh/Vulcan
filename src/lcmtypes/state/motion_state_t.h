/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_MOTION_STATE_T_H
#define LCMTYPES_MOTION_STATE_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_motion_state_t.h>

namespace vulcan
{
struct motion_state_t;

namespace lcm
{

const std::string MOTION_STATE_CHANNEL("STATE_MOTION_STATE");

void convert_lcm_to_vulcan(const vulcan_lcm_motion_state_t& stateMessage, motion_state_t& state);
void convert_vulcan_to_lcm(const motion_state_t& state, vulcan_lcm_motion_state_t& stateMessage);

void publish_data(lcm_t* lcm, const motion_state_t& state, std::string channel = MOTION_STATE_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const motion_state_t&, const std::string&, void*), void* userdata, std::string channel = MOTION_STATE_CHANNEL);

}
}

#endif // LCMTYPES_MOTION_STATE_T_H
