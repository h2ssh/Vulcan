/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     filters.cpp
* \author   Collin Johnson
*
* Definition of various filters to apply to weighted gateways:
*
*   - filter_out_of_map_gateways
*   - filter_generated_gateways
*/

#include <hssh/local_topological/area_detection/gateways/filters.h>
#include <hssh/local_topological/area_detection/gateways/gateway_utils.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <utils/algorithm_ext.h>
#include <algorithm>

namespace vulcan
{
namespace hssh
{

bool is_unique_gateway(const WeightedGateway& lhs, const WeightedGateway& rhs);
bool do_gateways_intersect(const WeightedGateway& lhs, const WeightedGateway& rhs);


std::vector<WeightedGateway> filter_out_of_map_gateways(const std::vector<WeightedGateway>& gateways,
                                                        const VoronoiSkeletonGrid& skeleton)
{
    std::vector<WeightedGateway> inMapGateways;

    // If either endpoint is not in the grid, then throw out the gateway
    // If the center is no longer in free space, along throw it out, as the boundary of the functional LPM has moved
    std::copy_if(gateways.begin(),
                 gateways.end(),
                 std::back_inserter(inMapGateways),
                 [&skeleton](const WeightedGateway& g) {
        return skeleton.isCellInGrid(utils::global_point_to_grid_cell_round(g.gateway.boundary().a, skeleton))
            && skeleton.isCellInGrid(utils::global_point_to_grid_cell_round(g.gateway.boundary().b, skeleton))
            && !(skeleton.getClassification(utils::global_point_to_grid_cell_round(g.gateway.center(), skeleton)) & SKELETON_CELL_UNKNOWN);
    });

    return inMapGateways;
}


std::vector<WeightedGateway> filter_generated_gateways(const std::vector<WeightedGateway>& gateways,
                                                       const VoronoiSkeletonGrid& skeleton)
{
    auto filtered = gateways;

    std::sort(filtered.begin(), filtered.end(), std::greater<WeightedGateway>());

    auto end = filtered.end();

    for(auto gatewayIt = filtered.begin(); gatewayIt != end; ++gatewayIt)
    {
        end = std::remove_if(gatewayIt+1, end, [gatewayIt](const WeightedGateway& g) {
            return do_gateways_intersect(*gatewayIt, g) // intersecting gateways can never exist
                || !is_unique_gateway(*gatewayIt, g);   // all gateways must be considered distinct
        });
    }

    end = filtered.erase(end, filtered.end());

    return filtered;
}


bool is_gateway_boundary_valid(const Gateway& g, const VoronoiSkeletonGrid& skeleton)
{
    // A boundary is valid if:
    //  - the cell boundary matches the translated boundary
    //  - both endpoints are on obstacle cells
    //  - the center is on a skeleton cell
    //  - all boundary cells are stored

    const uint8_t kValidEndMask = SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER;

    auto cellBoundary = g.cellBoundary();
    cell_t cellCenter = g.skeletonCell();

    auto skeletonCenter = utils::global_point_to_grid_cell_round(g.center(), skeleton);
    auto skeletonBoundary = Line<int>(utils::global_point_to_grid_cell_round(g.boundary().a, skeleton),
                                            utils::global_point_to_grid_cell_round(g.boundary().b, skeleton));

    bool skeletonIsSame = (skeletonCenter == cellCenter) && (skeletonBoundary == cellBoundary);
    bool cellsAreValid = (skeleton.getClassification(cellBoundary.a.x, cellBoundary.a.y) & kValidEndMask)
        && (skeleton.getClassification(cellBoundary.b.x, cellBoundary.b.y) & kValidEndMask)
        && (skeleton.getClassification(cellCenter.x, cellCenter.y) & (SKELETON_CELL_REDUCED_SKELETON))
        && (g.sizeCells() > 0);

    if(!skeletonIsSame && cellsAreValid)
    {
        std::cerr << "WARNING: Would have accepted bad gateway: Old: " << cellBoundary << " New: " << skeletonBoundary
            << '\n';
    }

    return skeletonIsSame && cellsAreValid;
}


bool is_unique_gateway(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
//     double maxLength = std::max(lhs.gateway.length(), rhs.gateway.length());
//     double minLength = std::min(lhs.gateway.length(), rhs.gateway.length());
//
//     return !lhs.gateway.isSimilarTo(rhs.gateway) || (minLength / maxLength < 0.8);
    return !lhs.gateway.isSimilarTo(rhs.gateway);
}


bool do_gateways_intersect(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
    // If two gateways have the same ending cell, they can't intersect
    if((lhs.gateway.cellBoundary().a == rhs.gateway.cellBoundary().a)
        || (lhs.gateway.cellBoundary().a == rhs.gateway.cellBoundary().b)
        || (lhs.gateway.cellBoundary().b == rhs.gateway.cellBoundary().a)
        || (lhs.gateway.cellBoundary().b == rhs.gateway.cellBoundary().b))
    {
        return false;
    }

    Point<int> intersection;
    if(line_segment_intersection_point(lhs.gateway.cellBoundary(), rhs.gateway.cellBoundary(), intersection))
    {
        bool isLhsEndpoint = (lhs.gateway.cellBoundary().a == intersection)
            && (lhs.gateway.cellBoundary().b == intersection);
        bool isRhsEndpoint = (rhs.gateway.cellBoundary().a == intersection)
            && (rhs.gateway.cellBoundary().b == intersection);
        // If an intersection, check if it is an endpoint of both lines, in which case the intersection doesn't matter
        return !isLhsEndpoint || !isRhsEndpoint;
    }

    // No intersection
    return false;
}

} // namespace hssh
} // namespace vulcan
