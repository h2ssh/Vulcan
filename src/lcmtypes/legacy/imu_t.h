/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_SENSORS_IMU_T_H
#define LCMTYPES_SENSORS_IMU_T_H

#include <string>
#include "lcmtypes/vulcan_lcm_imu_t.h"

namespace vulcan
{
struct imu_data_t;

namespace lcm
{

const std::string IMU_DATA_CHANNEL("SENSOR_IMU");

void convert_lcm_to_vulcan(const vulcan_lcm_imu_t& imuMessage, imu_data_t& imuData);
void convert_vulcan_to_lcm(const imu_data_t& imuData, vulcan_lcm_imu_t& imuMessage);

void publish_data(lcm_t* lcm, const imu_data_t& imuData, std::string channel = IMU_DATA_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const imu_data_t&, const std::string&, void*), void* userdata, std::string channel = IMU_DATA_CHANNEL);

}
}

#endif // LCMTYPES_SENSORS_IMU_T_H
