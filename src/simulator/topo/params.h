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
* Declaration of params structs for the topo_map_simulator module and the function for loading the parameters.
*/

#ifndef SIMULATOR_TOPO_PARAMS_H
#define SIMULATOR_TOPO_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace simulator
{

struct topological_map_simulator_params_t
{
    // Nothing for now
};

struct topo_map_simulator_params_t
{
    topological_map_simulator_params_t simParams;
};

/**
* load_topo_map_simulator_params loads the parameters for the topo_map_simulator module from
* the provided configuration file.
*
* \param    config              ConfigFile with the parameters to load
* \return   Parameters pulled from the config file.
*/
topo_map_simulator_params_t load_topo_map_simulator_params(const utils::ConfigFile& config);

}
}

#endif // SIMULATOR_TOPO_PARAMS_H
