/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.cpp
 * \author   Collin Johnson
 *
 * Definition of the various config->params conversion functions.
 */

#include "logging/logplayer/params.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace logplayer
{

const std::string CHANNEL_HEADING("DataChannels");
const std::string IMU_CHANNEL_KEY("imu_channel");
const std::string ODOMETRY_CHANNEL_KEY("odometry_channel");
const std::string LASER_CHANNEL_KEY("laser_channel");


data_channels_t load_data_channels(const utils::ConfigFile& config);


log_player_params_t load_log_player_params(const utils::ConfigFile& config)
{
    log_player_params_t params;

    params.channels = load_data_channels(config);

    return params;
}


data_channels_t load_data_channels(const utils::ConfigFile& config)
{
    data_channels_t channels;

    channels.imuChannel = config.getValueAsString(CHANNEL_HEADING, IMU_CHANNEL_KEY);
    channels.odometryChannel = config.getValueAsString(CHANNEL_HEADING, ODOMETRY_CHANNEL_KEY);
    channels.laserChannel = config.getValueAsString(CHANNEL_HEADING, LASER_CHANNEL_KEY);

    return channels;
}

}   // namespace logplayer
}   // namespace vulcan
