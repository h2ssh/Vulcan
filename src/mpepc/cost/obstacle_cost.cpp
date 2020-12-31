/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     obstacle_cost.cpp
 * \author   Collin Johnson
 *
 * Definition of obstacle_distance_cost.
 */

#include "mpepc/cost/obstacle_cost.h"
#include "core/float_comparison.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include <cassert>
#include <cstdint>

namespace vulcan
{
namespace mpepc
{

void obstacle_distance_cost(const obstacle_distance_cost_params_t& params,
                            const planning_environment_t& env,
                            CostMap& costs)
{
    assert(env.dists);

    for (int y = 0, yEnd = costs.getHeightInCells(); y < yEnd; ++y) {
        for (int x = 0, xEnd = costs.getWidthInCells(); x < xEnd; ++x) {
            int32_t cost = 0;
            float dist = env.dists->getObstacleDistance(Point<int>(x, y));

            // If on an obstacle, then it is a collision.
            // Extend one cell beyond the actual wall to account for little errors in the social norm cost
            if (dist <= costs.metersPerCell()) {
                cost = 2 * kMinCollisionCost;
            }
            // If within the robot radius, also a collision, so the robot can't go there
            else if (dist <= params.robotRadius) {
                cost = kMinCollisionCost;
            }
            // Otherwise, the cost is in the falloff region
            else if (dist < params.obstacleCostRadius) {
                cost = params.alphaCost * kStraightCellDist
                  * std::pow((params.obstacleCostRadius - dist) / params.obstacleCostRadius, params.betaExp);
            } else {
                cost = 0;
            }

            costs.addCost(x, y, cost);
        }
    }
}

}   // namespace mpepc
}   // namespace vulcan
