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
 * Declaration of multi_floor_mapper_params_t and load_multi_floor_mapper_params().
 */


#ifndef HSSH_LOCAL_METRIC_MULTIFLOOR_PARAMS_H
#define HSSH_LOCAL_METRIC_MULTIFLOOR_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace hssh
{

struct multi_floor_mapper_params_t
{
    std::string floorFilename;
    double minDistanceBetweenFloors;
    bool shouldMatchElevators;
};

/**
 * load_multi_floor_mapper_params loads a params struct from the provided ConfigFile.
 *
 * \param    config          ConfigFile with the parameters to load
 * \return   New params struct pulled from the config.
 */
multi_floor_mapper_params_t load_multi_floor_mapper_params(const utils::ConfigFile& config);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_METRIC_MULTIFLOOR_PARAMS_H
