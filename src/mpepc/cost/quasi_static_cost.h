/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     quasi_static_cost.h
* \author   Collin Johnson
* 
* Declaration of a cost function for costs associated with dynamic objects being treated as quasi-static.
*/

#ifndef MPEPC_COSTS_QUASI_STATIC_COST_H
#define MPEPC_COSTS_QUASI_STATIC_COST_H

#include "mpepc/types.h"
#include "mpepc/cost/cost_map.h"

namespace vulcan
{
namespace mpepc
{
    
struct quasi_static_cost_params_t
{
    float costRadius;
    float costPeak;
    float costSlope;
};

/**
* quasi_static_cost adds a small amount of cost to the cost map that is associated with dynamic objects in the
* environment. Each quasi-static point is the center of some object. These points are expanded by a fixed amount and
* falloff to zero cost. Then, a linear amount of cost attaching the object to the robot frame is added in order to
* eliminate the local minimum created right in front of an object.
*/
void quasi_static_cost(const std::vector<Point<float>>& quasiStaticPoints,
                       const quasi_static_cost_params_t& params, 
                       const planning_environment_t& env, 
                       CostMap& costs);

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_COSTS_QUASI_STATIC_COST_H
