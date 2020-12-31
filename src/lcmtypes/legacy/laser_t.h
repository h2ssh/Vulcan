/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_LASER_T_H
#define LCMTYPES_LASER_T_H

#include <string>
#include <vector>
#include "core/laser_scan.h"
#include "lcmtypes/vulcan_lcm_laser_t.h"
#include "lcmtypes/vulcan_lcm_laser_3dof_t.h"
#include "lcmtypes/vulcan_lcm_laser_old_t.h"
#include "lcmtypes/vulcan_lcm_laser_with_intensity_t.h"

namespace vulcan
{
namespace lcm
{

const std::vector<std::string> LASER_SCAN_CHANNELS = {"SENSOR_LASER_FRONT_6DOF", "SENSOR_LASER_BACK_6DOF"};

void convert_lcm_to_vulcan(const vulcan_lcm_laser_t& laserMessage, polar_laser_scan_t& scan);
void convert_vulcan_to_lcm(const polar_laser_scan_t& scan, vulcan_lcm_laser_t& laserMessage);

void convert_lcm_to_vulcan(const vulcan_lcm_laser_3dof_t& laserMessage, polar_laser_scan_t& scan);
void convert_lcm_to_vulcan(const vulcan_lcm_laser_old_t& laserMessage, polar_laser_scan_t& scan);
void convert_lcm_to_vulcan(const vulcan_lcm_laser_with_intensity_t& laserMessage, polar_laser_scan_t& scan);

void publish_data(lcm_t* lcm, const polar_laser_scan_t& scan, std::string channel = std::string(""));   // No default because there are multiple channels possible
void subscribe_to_message(lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, std::string channel = std::string("")); // default subscribes to all channels

void initialize_laser_message(vulcan_lcm_laser_t& laserMessage);
void free_laser_message      (vulcan_lcm_laser_t& laserMessage);

}
}

#endif // LCMTYPES_LASER_T_H
