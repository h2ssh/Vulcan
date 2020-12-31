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
* Definition of load_goal_params and associate functions for turning a config file
* into params structs.
*/

#include "planner/goal/params.h"
#include <cassert>
#include "utils/config_file.h"

namespace vulcan
{
namespace planner
{

goal_params_t load_goal_params(const utils::ConfigFile& config)
{
    goal_params_t params;

    return params;
}

} // namespace planner
} // namespace vulcanh
