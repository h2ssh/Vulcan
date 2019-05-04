/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Declaration of the various params structs for the logplayer and the load_logplayer_params function.
*/

#ifndef LOGPLAYER_PARAMS_H
#define LOGPLAYER_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace logplayer
{

struct data_channels_t
{
    std::string imuChannel;
    std::string odometryChannel;
    std::string laserChannel;
};

struct log_player_params_t
{
    data_channels_t channels;
};

/**
* load_log_player_params loads the parameters for a given run of the LogPlayer.
*
* \param    config          ConfigFile from which to pull the parameters
* \return   Parameters read from the config file.
*/
log_player_params_t load_log_player_params(const utils::ConfigFile& config);

}
}

#endif // LOGPLAYER_PARAMS_H
