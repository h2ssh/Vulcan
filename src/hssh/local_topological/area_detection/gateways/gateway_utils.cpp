/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_utils.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for constructing gateways:
 *
 *   - are_gateways_intersecting
 *   - are_gatway_angle_close
 *   - is_gateway_untraversable
 *   - select_straightest_gateway_boundary
 */

#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/gateways/endpoint_validator.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include "utils/ray_tracing.h"
#include <array>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

const uint8_t kValidEndpointMask = SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER;
const uint8_t kValidCenterMask = SKELETON_CELL_REDUCED_SKELETON;


cell_t skeleton_cell_between_endpoints(const Line<int>& cellBoundary, const VoronoiSkeletonGrid& grid);
cell_t nearest_valid_location(cell_t start, int searchRadius, uint8_t validMask, const VoronoiSkeletonGrid& grid);


bool are_gateways_intersecting(const Gateway& lhs, const Gateway& rhs)
{
    Point<int> intersection;

    return lhs.intersectsWithCellBoundary(rhs.cellBoundary(), intersection);
}


bool are_gateway_angles_close(const Gateway& lhs,
                              const Gateway& rhs,
                              double closeAngleThreshold,
                              double endpointDistanceThreshold)
{
    // Arrange the gateways so their closest endpoints are in the 0 index of an array, which allows the calculated angle
    // for each to show if they are really pointing the same direction or not.
    auto lhsBoundary = lhs.boundary();
    auto rhsBoundary = rhs.boundary();

    std::array<Point<double>, 2> lhsEnds = {{lhsBoundary.a, lhsBoundary.b}};
    std::array<Point<double>, 2> rhsEnds = {{rhsBoundary.a, rhsBoundary.b}};

    int closestLhsIndex = 0;
    int closestRhsIndex = 0;
    float minDistance = 10000000000.0f;

    for (int n = 0; n < 2; ++n) {
        for (int i = 0; i < 2; ++i) {
            float endpointDistance = distance_between_points(lhsEnds[n], rhsEnds[i]);
            if (endpointDistance < minDistance) {
                minDistance = endpointDistance;
                closestLhsIndex = n;
                closestRhsIndex = i;
            }
        }
    }

    if (closestLhsIndex != 0) {
        std::swap(lhsEnds[0], lhsEnds[1]);
    }

    if (closestRhsIndex != 0) {
        std::swap(rhsEnds[0], rhsEnds[1]);
    }

    bool areAnglesClose =
      std::abs(angle_diff(angle_to_point(lhsEnds[0], lhsEnds[1]), angle_to_point(rhsEnds[0], rhsEnds[1])))
      < closeAngleThreshold;
    bool areEndpointsClose = minDistance < endpointDistanceThreshold;

    return areAnglesClose && areEndpointsClose;
}


bool is_gateway_traversable(const Gateway& gateway, double robotLength, const VoronoiSkeletonGrid& skeleton)
{
    // Trace through the skeleton from the gateway cell. Go up to lengthInCells, as anything over that length isn't
    // needed. Go through as many branches as needed -- can't go through more than lengthInCells, so that's sufficient
    // Need to follow the full skeleton because a dead end might not cram in the back of a partially seen office
    std::size_t lengthInCells = robotLength * skeleton.cellsPerMeter();
    auto traces = trace_voronoi_graph(gateway.skeletonCell(), skeleton, lengthInCells, lengthInCells, false);

    // Find the longest trace for each side of the gateway
    CellToTypeMap<std::size_t> cellToMaxLength;
    for (auto& t : traces.traces) {
        if (t.points.size() > 1) {
            if (cellToMaxLength.find(t.points[1]) != cellToMaxLength.end()) {
                cellToMaxLength[t.points[1]] = std::max(cellToMaxLength[t.points[1]], t.points.size());
            } else {
                cellToMaxLength[t.points[1]] = t.points.size();
            }
        }
    }

    // See how many lengths along the reduced skeleton are long enough. Gateways can't exist at reduced junctions, but
    // can be at full skeleton junctions, so only check the reduced skeleton branches to see if they are long enough.
    int numLongEnough = 0;
    for (auto lengths : cellToMaxLength) {
        if ((skeleton.getClassification(lengths.first.x, lengths.first.y) & SKELETON_CELL_REDUCED_SKELETON)
            && (lengths.second == lengthInCells)) {
            ++numLongEnough;
        }
    }

    // Both branches of the reduced skeleton need to be long enough
    return numLongEnough == 2;
}


void gateway_boundary_cells(const Gateway& gateway, const VoronoiSkeletonGrid& grid, CellVector& boundaryCells)
{
    auto center = gateway.skeletonCell();
    auto boundary = gateway.cellBoundary();
    auto deltaA = boundary.a - center;
    auto deltaB = boundary.b - center;

    // Start all endpoints slightly inside the associated cell, which helps keep truncation
    // errors in the ray trace from causing the gateway to fall in the wrong cell
    Point<double> adjustedCenter(center.x + 0.01, center.y + 0.01);

    auto boundaryA =
      Line<double>(adjustedCenter, Point<double>(adjustedCenter.x + deltaA.x, adjustedCenter.y + deltaA.y));
    auto boundaryB =
      Line<double>(adjustedCenter, Point<double>(adjustedCenter.x + deltaB.x, adjustedCenter.y + deltaB.y));

    // Trace along the gateway boundary to get all cells it passes through
    utils::find_cells_along_line(boundaryA, grid, std::back_inserter(boundaryCells));
    utils::find_cells_along_line(boundaryB, grid, std::back_inserter(boundaryCells));

    utils::erase_unique(boundaryCells);
}


float gateway_cell_perimeter(const Gateway& gateway, float metersPerCell)
{
    // Iterate along the boundary. If a cell is four-way connected, then it adds 1 to perim. If 8-way connected,
    // it adds 2

    if (gateway.sizeCells() == 1) {
        return metersPerCell;
    }

    return cell_vector_perimeter(gateway.beginCells(), gateway.endCells(), metersPerCell);
}


double gateway_normal_from_source_cells(cell_t cell, const VoronoiSkeletonGrid& skeleton)
{
    double maxAngle = 0.0;
    double maxNormal = 0.0;

    auto sources = skeleton.getSourceCells(cell.x, cell.y);

    for (std::size_t n = 0; n < sources.size(); ++n) {
        for (std::size_t m = n + 1; m < sources.size(); ++m) {
            // Angular separation is relative to the skeleton cell
            double angleBetweenSources = std::abs(angle_between_points(sources[n], sources[m], cell));
            if (angleBetweenSources > maxAngle) {
                maxAngle = angleBetweenSources;
                maxNormal = angle_sum(angle_to_point(sources[n], sources[m]), PI_F);
            }
        }
    }

    return maxNormal;
}


boost::optional<Gateway> create_gateway_at_cell(cell_t cell,
                                                double normal,
                                                int32_t id,
                                                const VoronoiSkeletonGrid& skeleton,
                                                double maxExtraLength)
{
    auto cellBoundary = gateway_boundary_line_at_cell(cell, normal, skeleton, maxExtraLength);

    if (!cellBoundary) {
        return boost::none;
    }

    // If the cell isn't on the reduced skeleton, then find it
    if (~skeleton.getClassification(cell.x, cell.y) & SKELETON_CELL_REDUCED_SKELETON) {
        cell = skeleton_cell_between_endpoints(*cellBoundary, skeleton);

        if (!skeleton.isCellInGrid(cell)) {
            return boost::none;
        }
    }

    // Return a valid gateway that satisfies the post-condition
    Gateway gateway(skeleton.getTimestamp(), id, *cellBoundary, cell, skeleton);

    //     bool foundSkeleton = false;
    //     for(auto& cell : boost::make_iterator_range(gateway.beginCells(), gateway.endCells()))
    //     {
    //         if(skeleton.getClassification(cell.x, cell.y) & SKELETON_CELL_REDUCED_SKELETON)
    //         {
    //             if(foundSkeleton)
    //             {
    //                 std::cout << "Filtering gateway at " << gateway.skeletonCell() << " Double-crosser!";
    //                 return boost::none;
    //             }
    //
    //             foundSkeleton = true;
    //         }
    //     }

    return gateway;
}


boost::optional<Line<int>>
  gateway_boundary_line_at_cell(cell_t cell, double normal, const VoronoiSkeletonGrid& skeleton, double maxExtraLength)
{
    // The endpoints of the gateway are in the +/- pi/2 directions. Trace to the nearest obstacle.
    const double kGatewayLengthAddition = maxExtraLength;
    Line<int> cellBoundary;

    cellBoundary.a =
      utils::trace_ray_until_condition(cell,
                                       normal + M_PI_2,
                                       skeleton.getMetricDistance(cell.x, cell.y) + kGatewayLengthAddition,
                                       skeleton,
                                       VoronoiSkeletonTerminationFunc(kValidEndpointMask));
    cellBoundary.b =
      utils::trace_ray_until_condition(cell,
                                       normal - M_PI_2,
                                       skeleton.getMetricDistance(cell.x, cell.y) + kGatewayLengthAddition,
                                       skeleton,
                                       VoronoiSkeletonTerminationFunc(kValidEndpointMask));

    // If either cell boundary isn't on an unknown or occupied cell, then the ray trace failed to satisfy the condition
    // so no valid gateway exists.
    // If both ends are the same, it also can't be a valid gateway.
    if (!(skeleton.getClassification(cellBoundary.a.x, cellBoundary.a.y) & kValidEndpointMask)
        || !(skeleton.getClassification(cellBoundary.b.x, cellBoundary.b.y) & kValidEndpointMask)
        || (cellBoundary.a == cellBoundary.b)) {
        return boost::none;
    }

    return cellBoundary;
}


boost::optional<Gateway> create_gateway_between_sources(const CellVector& sources,
                                                        cell_t skeletonCell,
                                                        int32_t id,
                                                        const VoronoiSkeletonGrid& skeleton,
                                                        const VoronoiIsovistField& isovists)
{
    // Weird situation can result in a single source cell, for which there is obviously no
    if (sources.size() < 2) {
        return boost::none;
    }

    // Find the boundary amongst the gateways that is closest to the expected length, given the location of the
    // skeleton cell
    double optimalLength = 2.0 * skeleton.getMetricDistance(skeletonCell.x, skeletonCell.y) * skeleton.cellsPerMeter();
    Line<int> boundary(sources[0], sources[1]);
    for (std::size_t n = 0; n < sources.size(); ++n) {
        for (std::size_t m = n + 1; m < sources.size(); ++m) {
            double len = distance_between_points(sources[n], sources[m]);
            if (std::abs(len - optimalLength) < std::abs(length(boundary) - optimalLength)) {
                boundary.a = sources[n];
                boundary.b = sources[m];
            }
        }
    }

    // Construct a gateway from this boundary
    auto gatewaySkeleton = skeleton_cell_between_endpoints(boundary, skeleton);
    // If a valid cell was found, then create the gateway
    if ((gatewaySkeleton.x >= 0) && (gatewaySkeleton.y >= 0)) {
        assert(skeleton.getClassification(gatewaySkeleton.x, gatewaySkeleton.y) & kValidCenterMask);
        return Gateway(skeleton.getTimestamp(), id, boundary, gatewaySkeleton, isovists, skeleton);
    }
    // Otherwise no gateway exists
    else {
        return boost::none;
    }
}


boost::optional<Gateway>
  adjust_gateway_for_new_skeleton(const Gateway& gateway, const VoronoiSkeletonGrid& skeleton, int maxSearchRadius)
{
    Line<int> newBoundary;
    newBoundary.a = nearest_valid_location(utils::global_point_to_grid_cell_round(gateway.boundary().a, skeleton),
                                           maxSearchRadius,
                                           kValidEndpointMask,
                                           skeleton);
    newBoundary.b = nearest_valid_location(utils::global_point_to_grid_cell_round(gateway.boundary().b, skeleton),
                                           maxSearchRadius,
                                           kValidEndpointMask,
                                           skeleton);

    if (!skeleton.isCellInGrid(newBoundary.a) || !skeleton.isCellInGrid(newBoundary.b)) {
        std::cout << "FAILED TO ADJUST BOUNDARY: Old:" << gateway.cellBoundary() << " New:" << newBoundary << '\n';
        return boost::none;
    }

    cell_t skeletonCell = skeleton_cell_between_endpoints(newBoundary, skeleton);

    if (!skeleton.isCellInGrid(skeletonCell)) {
        std::cout << "FAILED TO ADJUST SKELETON: Old:" << gateway.skeletonCell() << " New boundary:" << newBoundary
                  << " New skeleton: " << skeletonCell << '\n';
        return boost::none;
    }

    return Gateway(skeleton.getTimestamp(), gateway.id(), newBoundary, skeletonCell, skeleton);
}

// Return a cell along the boundary that is reduced skeleton, or a cell not in the map
cell_t skeleton_cell_between_endpoints(const Line<int>& cellBoundary, const VoronoiSkeletonGrid& grid)
{
    // Find all cells along the boundary
    CellVector boundaryCells;
    utils::find_cells_along_line(cellBoundary, grid, std::back_inserter(boundaryCells));

    // See if any cells are skeleton cells on the boundary
    auto skeletonIt = std::find_if(boundaryCells.begin(), boundaryCells.end(), [&grid](cell_t c) {
        return grid.getClassification(c.x, c.y) & SKELETON_CELL_REDUCED_SKELETON;
    });

    // If didn't find a skeleton cell, then check for a four-way connected cell adjacent to the boundary. Due to
    // discretization, it is possible to step by the desired skeleton cell
    if (skeletonIt == boundaryCells.end()) {
        // If one of the skeleton cells has a neighbor cell with the desired label, then use it.
        NeighborArray neighbors;
        for (auto cell : boundaryCells) {
            int num = neighbor_cells_with_classification(cell, kValidCenterMask, grid, EIGHT_WAY, neighbors);

            if (num > 0) {
                return neighbors[0];
            }
        }

        std::cout << "Failed to find a reduced skeleton cell along cells: ";
        std::copy(boundaryCells.begin(), boundaryCells.end(), std::ostream_iterator<cell_t>(std::cout, " "));
        std::cout << '\n';
        return cell_t(-1, -1);
    }
    // Otherwise, assign the cell to be the skeleton cell that was found
    else {
        return *skeletonIt;
    }
}

// Return a cell not in the grid if no valid location is within the search radius
cell_t nearest_valid_location(cell_t start, int searchRadius, uint8_t validMask, const VoronoiSkeletonGrid& grid)
{
    // If the start cell is valid, then we're done!
    if (grid.getClassification(start) & kValidEndpointMask) {
        return start;
    }

    // Otherwise run a search through all the cells within the search radius. Check the cells in order of increasing
    // radius away from the starting cell (approximately) by searching rectangles of cells with increasing perimeter
    // centered at the start cell
    std::vector<int> indices;
    indices.push_back(0);
    for (int n = 1; n <= searchRadius; ++n) {
        indices.push_back(n);
        indices.push_back(-n);
    }

    for (int y : indices) {
        for (int x = -std::abs(y), xEnd = std::abs(y); x <= xEnd; ++x) {
            cell_t newCell(start.x + x, start.y + y);
            if (grid.getClassification(newCell.x, newCell.y) & kValidEndpointMask) {
                return newCell;
            }
        }

        // The above loop won't check cells along the line y = 0. Do that check separately right here.
        cell_t newCell(start.x + y, start.y);
        if (grid.getClassification(newCell.x, newCell.y) & kValidEndpointMask) {
            return newCell;
        }
    }

    return cell_t(-1, -1);
}

}   // namespace hssh
}   // namespace vulcan
