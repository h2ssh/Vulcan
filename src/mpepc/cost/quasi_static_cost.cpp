/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     quasi_static_cost.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of quasi_static_cost.
 */

#include "mpepc/cost/quasi_static_cost.h"

namespace vulcan
{
namespace mpepc
{

void quasi_static_cost(const std::vector<Point<float>>& quasiStaticPoints,
                       const quasi_static_cost_params_t& params,
                       const planning_environment_t& env,
                       CostMap& costs)
{
    const int kCostWidthInCells = costs.cellsPerMeter() * params.costRadius;
    const int32_t kMaxCost = costs.cellsPerMeter() * params.costPeak;

    for (auto point : quasiStaticPoints) {
        auto gridPoint = utils::global_point_to_grid_cell(point, costs);

        // Compute and add cost to a circle of kCostWidthInCells around the point of interest
        for (int dy = -kCostWidthInCells; dy <= kCostWidthInCells; dy++) {
            for (int dx = -kCostWidthInCells; dx <= kCostWidthInCells; dx++) {
                if (dx * dx + dy * dy < kCostWidthInCells * kCostWidthInCells) {
                    Point<int> candidateCell(gridPoint.x + dx, gridPoint.y + dy);
                    if (costs.isCellInGrid(candidateCell)) {
                        int32_t fallOff = costs.cellsPerMeter() * std::sqrt((dx * dx) + (dy * dy)) * params.costSlope;
                        int32_t cost = kMaxCost - fallOff;

                        if (cost > 0) {
                            int32_t oldValue = costs.getValue(candidateCell.x, candidateCell.y);
                            int32_t newValue = (cost > oldValue) ? cost : oldValue;   // use whichever is the largest
                            costs.addCost(candidateCell.x, candidateCell.y, newValue);
                        }
                    }
                }
            }
        }

        // compute and add costs along the line of sight between the robot point and the target point
        // this can be understood as (kind of...?) a simplified velocity obstacle implemented over grid.
        const double kStartCirclingAroundObjectDistance = 2.0;

        float deltaX = point.x - env.robotState.pose.x;
        float deltaY = point.x - env.robotState.pose.y;
        float lineOfSightAngle = atan2(deltaY, deltaX);

        bool isObjectClose =
          deltaX * deltaX + deltaY * deltaY < kStartCirclingAroundObjectDistance * kStartCirclingAroundObjectDistance;
        bool isObjectInFront = fabs(angle_diff(lineOfSightAngle, env.robotState.pose.theta)) < M_PI / 2;

        if (isObjectClose && isObjectInFront) {
            for (float dist = 0; dist < kStartCirclingAroundObjectDistance; dist += costs.metersPerCell()) {
                float x = env.robotState.pose.x + (dist * cos(lineOfSightAngle));
                float y = env.robotState.pose.x + (dist * sin(lineOfSightAngle));

                Point<float> candidatePoint(x, y);
                Point<int> candidateCell = utils::global_point_to_grid_cell(candidatePoint, costs);

                int32_t oldValue = costs.getValue(candidateCell.x, candidateCell.y);
                int32_t newValue = oldValue + kMaxCost;
                costs.addCost(candidateCell.x, candidateCell.y, newValue);
            }
        }
    }
}

}   // namespace mpepc
}   // namespace vulcan
