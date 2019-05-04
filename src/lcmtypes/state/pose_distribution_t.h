/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_POSE_DISTRIBUTION_T_H
#define LCMTYPES_POSE_DISTRIBUTION_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_pose_distribution_t.h>

namespace vulcan
{
struct pose_distribution_t;

namespace lcm
{

const std::string POSE_DISTRIBUTION_CHANNEL("STATE_POSE_DISTRIBUTION");

void convert_lcm_to_vulcan(const vulcan_lcm_pose_distribution_t& poseMessage, pose_distribution_t& pose);
void convert_vulcan_to_lcm(const pose_distribution_t& pose, vulcan_lcm_pose_distribution_t& poseMessage);

void publish_data(lcm_t* lcm, const pose_distribution_t& pose, std::string channel = POSE_DISTRIBUTION_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const pose_distribution_t&, const std::string&, void*), void* userdata, std::string channel = POSE_DISTRIBUTION_CHANNEL);

}
}

#endif // LCMTYPES_POSE_DISTRIBUTION_T_H
