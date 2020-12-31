/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     map_optimizer.cpp
 * \author   Collin Johnson
 *
 * Definition of create_map_optimizer factory.
 */

#include "hssh/global_topological/mapping/map_optimizer.h"
#include "hssh/global_topological/mapping/lev_mar_optimizer.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<MapOptimizer> create_map_optimizer(const std::string& type, const map_optimizer_params_t& params)
{
    if (type == LEV_MAR_OPTIMIZER_TYPE) {
        return std::unique_ptr<MapOptimizer>(new LevMarOptimizer(params.levMarParams));
    } else {
        std::cerr << "ERROR:Unknown MapOptimizer type: " << type << std::endl;
        assert(false);
    }

    return std::unique_ptr<MapOptimizer>();
}

}   // namespace hssh
}   // namespace vulcan
