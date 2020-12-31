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
* Definition of functions for creating the params structs from a ConfigFile.
*/

#include "simulator/topo/params.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace simulator
{

topological_map_simulator_params_t load_simulator_params(const utils::ConfigFile& config);


topo_map_simulator_params_t load_topo_map_simulator_params(const utils::ConfigFile& config)
{
    topo_map_simulator_params_t params;

    params.simParams  = load_simulator_params(config);

    return params;
}


topological_map_simulator_params_t load_simulator_params(const utils::ConfigFile& config)
{
    topological_map_simulator_params_t params;

    return params;
}

} // namespace simulator
} // namespace vulcan
