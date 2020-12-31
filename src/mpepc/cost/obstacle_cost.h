/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     obstacle_cost.h
 * \author   Collin Johnson
 *
 * Declaration of a cost function for costs associated with being near obstacles.
 */

#ifndef MPEPC_COSTS_OBSTACLE_COST_H
#define MPEPC_COSTS_OBSTACLE_COST_H

#include "mpepc/cost/cost_map.h"
#include "mpepc/types.h"

namespace vulcan
{
namespace mpepc
{

struct obstacle_distance_cost_params_t
{
    float robotRadius;
    float obstacleCostRadius;
    float obstacleCost;

    float alphaCost;   ///< Max social normal cost -- alpha in equations
    float betaExp;     ///< Cost falloff exponent -- beta in ions
};

/**
 * obstacle_distance_cost computes the cost of being on an obstacle in the environment. The cost is infinite for all
 * obstacle cells in the map
 *
 * \pre  env.dists != nullptr
 * \param        params      Parameters controlling how distance to obstacles is integrated into the cost map
 * \param        env         Environment to compute obstacle distance costs for
 * \param[out]   costs       Costs to be incremented for the environment
 */
void obstacle_distance_cost(const obstacle_distance_cost_params_t& params,
                            const planning_environment_t& env,
                            CostMap& costs);

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_COSTS_OBSTACLE_COST_H
