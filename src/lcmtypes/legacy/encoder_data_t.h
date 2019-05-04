/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_SENSORS_ENCODER_DATA_T_H
#define LCMTYPES_SENSORS_ENCODER_DATA_T_H

#include <string>
#include <lcmtypes/vulcan_lcm_encoder_data_t.h>

namespace vulcan
{
struct encoder_data_t;

namespace lcm
{

const std::string ENCODERS_CHANNEL("SENSOR_ENCODERS");

void convert_lcm_to_vulcan(const vulcan_lcm_encoder_data_t& encodersMessage, encoder_data_t& encoders);
void convert_vulcan_to_lcm(const encoder_data_t& encoders, vulcan_lcm_encoder_data_t& encodersMessage);

void publish_data(lcm_t* lcm, const encoder_data_t& encoders, std::string channel = ENCODERS_CHANNEL);
void subscribe_to_message(lcm_t* lcm, void (*callback)(const encoder_data_t&, const std::string&, void*), void* userdata, std::string channel = ENCODERS_CHANNEL);

}
}

#endif // LCMTYPES_SENSORS_ENCODER_DATA_T_H
