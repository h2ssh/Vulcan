/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     social_norm_utils.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for converting topological structure into cost functions:
 *
 *   - nearest_skeleton_index
 *   - is_right_of_skeleton
 */

#include "mpepc/social/social_norm_utils.h"
#include "core/float_comparison.h"
#include "core/line.h"
#include "core/motion_state.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/area_detection/voronoi/search.h"
#include "hssh/local_topological/local_topo_map.h"
#include "math/boundary.h"
#include "mpepc/social/topo_agent.h"
#include "utils/algorithm_ext.h"

namespace vulcan
{
namespace mpepc
{

bool is_about_to_transition(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap, double transitionTime)
{
    // If no velocity, then definitely not about to transition.
    if (absolute_fuzzy_equal(agent.state.xVel, 0.0f) && absolute_fuzzy_equal(agent.state.yVel, 0.0f)) {
        return false;
    }

    // Grab the gateway of interest -- not in an area?
    const auto& area = topoMap.areaWithId(agent.areaId);
    if (!area) {
        return false;
    }

    const auto& gateways = area->gateways();

    auto gwyIt = std::find_if(gateways.begin(), gateways.end(), [&agent](auto& gwy) {
        return gwy.id() == agent.gatewayId;
    });

    // If the gateway isn't found (maybe no line boundary) obviously not about to cross it
    if (gwyIt == gateways.end()) {
        return false;
    }

    Line<double> trajLine(agent.state.x,
                          agent.state.y,
                          agent.state.x + (transitionTime * agent.state.xVel),
                          agent.state.y + (transitionTime * agent.state.yVel));

    return line_segments_intersect(trajLine, gwyIt->boundary());
}


topo_agent_t create_agent_for_robot(const motion_state_t& robotState, const hssh::LocalTopoMap& topoMap)
{
    topo_agent_t agent;
    agent.radius = 0.32;   // TODO: Grab from config
    agent.state.x = robotState.pose.x;
    agent.state.y = robotState.pose.y;
    agent.state.xVel = std::cos(robotState.pose.theta) * robotState.velocity.linear;
    agent.state.yVel = std::sin(robotState.pose.theta) * robotState.velocity.linear;

    auto area = topoMap.areaContaining(robotState.pose.toPoint());

    if (area) {
        agent.areaId = area->id();

        double bestProb = -1.0;
        int bestId = -1;

        // Use the maximum probability gateway from the area based on our current heading as the estimate for where
        // the robot is going. Actual goal information isn't available
        for (auto& gwy : area->gateways()) {
            auto range = math::boundary_heading_range(gwy.boundary(), robotState.pose.toPoint(), robotState.pose.theta);
            double prob = math::boundary_heading_probability(range, robotState.poseDistribution.uncertainty(2, 2));
            if (prob > bestProb) {
                bestId = gwy.id();
                bestProb = prob;
            }
        }

        agent.gatewayId = bestId;
    } else {
        agent.areaId = -1;
        agent.gatewayId = -1;

        std::cout << "WARNING: Robot is not currently in an area? " << robotState.pose << '\n';
    }

    return agent;
}


hssh::CellVector
  skeleton_cells_along_route(hssh::cell_t startCell, hssh::cell_t endCell, const hssh::VoronoiSkeletonGrid& skeleton)
{
    // The Voronoi skeleton is used to approximate the center of the path
    auto cellsAlongRoute =
      hssh::find_path_along_skeleton(startCell, endCell, hssh::SKELETON_CELL_REDUCED_SKELETON, skeleton);
    // Only care about the cells along the skeleton, not those needed to get to it
    utils::erase_remove_if(cellsAlongRoute.cells, [&skeleton](hssh::cell_t cell) {
        return !(skeleton.getClassificationNoCheck(cell.x, cell.y) & hssh::SKELETON_CELL_REDUCED_SKELETON);
    });

    return cellsAlongRoute.cells;
}


int nearest_skeleton_index(hssh::cell_t cell, const hssh::CellVector& skeleton, std::size_t hintIdx)
{
    if (skeleton.empty()) {
        return -1;
    }

    // Search out from the hint idx until a local minima is found
    hintIdx = std::min(hintIdx, skeleton.size() - 1);

    double minDist = squared_point_distance(skeleton[hintIdx], cell);
    double nextDist = (hintIdx + 1 < skeleton.size()) ? squared_point_distance(skeleton[hintIdx + 1], cell) : HUGE_VAL;
    double prevDist = (hintIdx > 0) ? squared_point_distance(skeleton[hintIdx - 1], cell) : HUGE_VAL;

    int searchDir = (prevDist < nextDist) ? -1 : 1;
    int limit = (prevDist < nextDist) ? -1 : skeleton.size();
    int minIdx = hintIdx;

    for (int n = minIdx + searchDir; n != limit; n += searchDir) {
        double dist = squared_point_distance(skeleton[n], cell);

        if (dist <= minDist) {
            minDist = dist;
            minIdx = n;
        }
        // Once distance starts going up for long enough, call it finished
        else if ((dist > minDist) && (std::abs(n - minIdx) > 10)) {
            break;
        }
    }

    //     int oldMin = std::distance(skeleton.begin(),
    //         std::min_element(skeleton.begin(), skeleton.end(), [&cell](hssh::cell_t lhs, hssh::cell_t rhs) {
    //             return squared_point_distance(lhs, cell) < squared_point_distance(rhs, cell);
    //     }));
    //
    //     int oldDist = squared_point_distance(cell, skeleton[oldMin]);
    //
    //     if(oldDist != minDist)
    //     {
    //         std::cerr << "WARNING: Optimized search found different min: Cell: " << cell << " Edge: " <<
    //         skeleton.front() << "->" << skeleton.back() << " Old: " << skeleton[oldMin]
    //             << " New: " << skeleton[minIdx] << " Diff: " << (std::sqrt(minDist) - std::sqrt(oldDist))
    //             << '\n';
    //     }

    return minIdx;
}


bool is_right_of_skeleton(hssh::cell_t cell, int skeletonIndex, const hssh::CellVector& skeleton)
{
    const int kLineIncr = 3;

    // Check if the cell is to the right of the line formed by the skeleton index and the next cell along the skeleton
    int nextIndex = skeletonIndex + kLineIncr;
    int prevIndex = skeletonIndex - kLineIncr;

    nextIndex = std::min(nextIndex, static_cast<int>(skeleton.size() - 1));
    prevIndex = std::max(prevIndex, 0);

    return !left_of_line(skeleton[prevIndex], skeleton[nextIndex], cell);
}


double normalized_position_path(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap)
{
    const auto& path = topoMap.pathSegmentWithId(agent.areaId);

    if (!path) {
        return -1.0;
    }

    const auto& gateways = path->gateways();

    auto gwyIt = std::find_if(gateways.begin(), gateways.end(), [&agent](const hssh::Gateway& gwy) {
        return gwy.id() == agent.gatewayId;
    });

    if (gwyIt == gateways.end()) {
        return -1.0;
    }

    // See which end is more misaligned with the agent, that is the entry end for determining which direction the agent
    // is moving towards the goal, which might not be the other end.

    auto plusGwy = path->plusTransition().gateway();
    auto minusGwy = path->minusTransition().gateway();

    hssh::cell_t startCell;

    // Assign the start to the opposite end as the goal
    if (plusGwy.id() == gwyIt->id()) {
        startCell = minusGwy.skeletonCell();
    } else if (minusGwy.id() == gwyIt->id()) {
        startCell = plusGwy.skeletonCell();
    }
    // If the goal isn't one of the ends, but a destination, then use the heading information to try and
    // determine which gateway to use.
    else {
        float agentHeading = std::atan2(agent.state.yVel, agent.state.xVel);
        float plusDiff = angle_diff_abs(agentHeading, plusGwy.direction());
        float minusDiff = angle_diff_abs(agentHeading, minusGwy.direction());

        startCell = (plusDiff > minusDiff) ? plusGwy.skeletonCell() : minusGwy.skeletonCell();
    }

    const auto& skeletonGrid = topoMap.voronoiSkeleton();

    auto skeletonCells = skeleton_cells_along_route(startCell, gwyIt->skeletonCell(), skeletonGrid);
    auto agentCell = utils::global_point_to_grid_cell(Point<float>(agent.state.x, agent.state.y), skeletonGrid);
    auto skelIndex = nearest_skeleton_index(agentCell, skeletonCells);

    // The path is narrowed by the agent radius because that is when it is adjacent to the wall, not when the center
    // is adjacent, which it couldn't be
    double pathRadius =
      skeletonGrid.getMetricDistance(skeletonCells[skelIndex].x, skeletonCells[skelIndex].y) - agent.radius;
    // Can't use skeleton directly because we only care about distance to relevant portion of skeleton, which doesn't
    // include all nooks and crannies along hallways. If there's a small alcove or a branch leading into an office,
    // then it might be wrong because the distance measured isn't to the same wall as the center of the skeleton
    double distToSkel = distance_between_points(agentCell, skeletonCells[skelIndex]) * skeletonGrid.metersPerCell();
    double lateralDistance = pathRadius - distToSkel;

    if (is_right_of_skeleton(agentCell, skelIndex, skeletonCells)) {
        // Right of the path it's the full distance minus the distance remaining to the nearest wall
        lateralDistance = pathRadius + distToSkel;
    }

    if (pathRadius > 0.0) {
        lateralDistance = std::max(lateralDistance, 0.0);
        return std::min(lateralDistance / (2.0 * pathRadius), 1.0);
    }

    std::cerr << "WARNING: Path radius was negative: " << pathRadius << '\n';
    return -1.0;
}


double normalized_position_gateway(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap)
{
    const auto& area = topoMap.areaWithId(agent.areaId);
    const auto& gateways = area->gateways();

    auto gwyIt = std::find_if(gateways.begin(), gateways.end(), [&agent](auto& gwy) {
        return gwy.id() == agent.gatewayId;
    });

    // If the gateway isn't found (maybe no line boundary) obviously not about to cross it
    if (gwyIt == gateways.end()) {
        std::cerr << "ERROR: Failed to find the gateway for the place example. It should exist.\n";
        return -1.0;
    }

    Line<double> trajLine(agent.state.x,
                          agent.state.y,
                          agent.state.x + agent.state.xVel,
                          agent.state.y + agent.state.yVel);

    Point<double> intersectionPoint;
    bool doIntersect = line_intersection_point(trajLine, gwyIt->boundary(), intersectionPoint);
    if (!doIntersect) {
        return -1.0;
    }

    double distance = 0.0;

    // The distance depends on which side of boundary. If on left, then endpointA is 0 distance
    if (left_of_line(trajLine, gwyIt->boundary().a)) {
        distance = distance_between_points(gwyIt->boundary().a, intersectionPoint);
    }
    // Otherwise endpointB is 0 distance
    else {
        distance = distance_between_points(gwyIt->boundary().b, intersectionPoint);
    }

    // Subtract radius to get distance to edge of robot
    distance = std::max(distance - agent.radius, 0.0);
    // Length is reduced by full robot diameter since it is closer to both walls
    distance /= std::max(gwyIt->length() - (agent.radius * 2.0), agent.radius * 2.0);

    return distance;
}

}   // namespace mpepc
}   // namespace vulcan
