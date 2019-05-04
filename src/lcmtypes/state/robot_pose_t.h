/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_ROBOT_POSE_T_H
#define LCMTYPES_ROBOT_POSE_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_robot_pose_t.h>

namespace vulcan
{
struct pose_t;

namespace lcm
{
    
const std::string POSE_CHANNEL("ROBOT_POSE");

void convert_lcm_to_vulcan(const vulcan_lcm_robot_pose_t& poseMessage, pose_t& pose);
void convert_vulcan_to_lcm(const pose_t& pose, vulcan_lcm_robot_pose_t& poseMessage);

void publish_data(lcm_t* lcm, const pose_t& pose, std::string channel = POSE_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const pose_t&, const std::string&, void*), void* userdata, std::string channel = POSE_CHANNEL);

}
}

#endif // LCMTYPES_ROBOT_POSE_T_H
