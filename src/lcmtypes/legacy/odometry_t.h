/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_SENSORS_ODOMETRY_T_H
#define LCMTYPES_SENSORS_ODOMETRY_T_H

#include <string>
#include "lcmtypes/vulcan_lcm_odometry_t.h"

namespace vulcan
{
struct odometry_t;

namespace lcm
{

const std::string ODOMETRY_CHANNEL("SENSOR_ODOMETRY");

void convert_lcm_to_vulcan(const vulcan_lcm_odometry_t& odometryMessage, odometry_t& odometry);
void convert_vulcan_to_lcm(const odometry_t& odometry, vulcan_lcm_odometry_t& odometryMessage);

void publish_data(lcm_t* lcm, const odometry_t& odometry, std::string channel = ODOMETRY_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const odometry_t&, const std::string&, void*), void* userdata, std::string channel = ODOMETRY_CHANNEL);

}
}

#endif // LCMTYPES_SENSORS_ODOMETRY_T_H
