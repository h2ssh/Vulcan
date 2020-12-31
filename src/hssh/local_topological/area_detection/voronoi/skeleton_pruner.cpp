/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     skeleton_pruner.cpp
 * \author   Collin Johnson
 *
 * Definition of SkeletonPruner.
 */

#include "hssh/local_topological/area_detection/voronoi/skeleton_pruner.h"
#include "hssh/local_topological/area_detection/voronoi/search.h"
#include "hssh/local_topological/area_detection/voronoi/skeleton_graph_rasterization.h"
#include "hssh/local_topological/area_detection/voronoi/skeleton_graph_reducer.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include <cmath>
#include <iostream>
#include <iterator>
#include <set>

// #define DEBUG_START_POINT
// #define DEBUG_EXIT_POINTS

namespace vulcan
{
namespace hssh
{

bool is_valid_start_cell(const cell_t& point, const VoronoiSkeletonGrid& grid, float minDistance, const pose_t& pose);
void prune_clumps(const SkeletonGraph& revg, VoronoiSkeletonGrid& grid);
bool is_cell_nub(cell_t cell, const VoronoiSkeletonGrid& grid, int maxLength = 3);


SkeletonPruner::SkeletonPruner(const skeleton_pruner_params_t& params)
: extractor(0.0)
, reducer(create_skeleton_graph_reducer(params.reducerType))
, minExitPointDistance(params.minExitPointDistance)
{
}


SkeletonPruner::~SkeletonPruner(void)
{
    // For std::unique_ptr
}


void SkeletonPruner::pruneSkeleton(VoronoiSkeletonGrid& grid,
                                   const pose_t& pose,
                                   const std::vector<Point<float>>& pointsOfInterest)
{
    std::set<cell_t> startCells;

    for (auto deadEnd : grid.getDeadEnds()) {
        if (is_valid_start_cell(deadEnd, grid, minExitPointDistance, pose)) {
            startCells.insert(deadEnd);
        }
    }

    if (startCells.empty()) {
        for (auto junction : grid.getJunctionsPoints()) {
            if (is_valid_start_cell(junction, grid, minExitPointDistance, pose)) {
                startCells.insert(junction);
            }
        }
    }

    for (auto& point : pointsOfInterest) {
        auto cell = utils::global_point_to_grid_cell_round(point, grid);
        // If the cell is there, just take it
        if (grid.getClassification(cell.x, cell.y) & SKELETON_CELL_SKELETON) {
            startCells.insert(cell);
        }
        // Otherwise check neighbors to see if any of them are a cell of interest
        else {
            NeighborArray neighbors;
            int num = neighbor_cells_equal_classification(cell, SKELETON_CELL_SKELETON, grid, EIGHT_WAY, neighbors);
            if (num > 0) {
                startCells.insert(neighbors[0]);
            }
        }
    }

    // Only prune if the start cell is inside the grid and marked as a skeleton cell,
    // otherwise there is no skeleton to be pruned away
    if (!startCells.empty()) {
#ifdef DEBUG_START_POINT
        std::cout << "DEBUG:Pruner:Start cells:\n";
        std::copy(startCells.begin(), startCells.end(), std::ostream_iterator<cell_t>(std::cout, " "));
        std::cout << '\n';
#endif

        // Extract the graph
        SkeletonGraph evg = extractor.extractGraph(startCells, grid);

        auto exitVertices = findExitVertices(grid, evg);

        // Reduce the graph
        const SkeletonGraph& reducedEVG = reducer->reduceSkeleton(evg, exitVertices);

        std::vector<SkeletonGraph> components = connected.connectedComponents(reducedEVG);

        auto maxGraphIt = std::max_element(components.begin(),
                                           components.end(),
                                           [](const SkeletonGraph& lhs, const SkeletonGraph& rhs) {
                                               return lhs.size() < rhs.size();
                                           });

        std::cout << "INFO: SkeletonPruner: Found " << components.size() << " subgraphs in the reduced EVG.\n"
                  << "INFO: SkeletonPruner: Reduced skeleton size:" << maxGraphIt->size() << '\n';

        // Rasterize the graph back onto the grid
        rasterize_graph_onto_grid(*maxGraphIt, grid);

        // Remove unnecessary cells from clumps that only served to connect now-pruned portions of the skeleton
        prune_clumps(*maxGraphIt, grid);

        // Find exit points
        findReducedExitPoints(grid, *maxGraphIt);


#ifdef DEBUG_EXIT_POINTS
        std::cout << "DEBUG:Pruner:Exit points:\n";
        for (auto exitIt = exitVertices.begin(), exitEnd = exitVertices.end(); exitIt != exitEnd; ++exitIt) {
            std::cout << (*exitIt)->point << ' ';
        }
        std::cout << '\n';
#endif
    } else {
        std::cerr << "ERROR:SkeletonPruner:No valid start cells for the pruning operation\n";
    }
}


std::vector<skeleton_graph_vertex_t*> SkeletonPruner::findExitVertices(const VoronoiSkeletonGrid& grid,
                                                                       const SkeletonGraph& evg)
{
    std::vector<skeleton_graph_vertex_t*> exitPoints;

    int width = grid.getWidthInCells();
    int height = grid.getHeightInCells();

    for (auto vertex : evg.getVertices()) {
        bool hasLongEdge = false;
        for (std::size_t n = 0; n < vertex->numAdjacent; ++n) {
            hasLongEdge = vertex->distance[n] > 2.0;
        }

        // If the vertex hits the edge of the map, it must be an exit point
        if ((vertex->point.x == 0) || (vertex->point.y == 0) || (vertex->point.x + 1 == width)
            || (vertex->point.y + 1 == height)) {
            exitPoints.push_back(vertex);
        }
        // If it is a long edge, then it should be added regardless of the exit distance
        else if (hasLongEdge) {
            exitPoints.push_back(vertex);
        }
        // If less than the min exit point distance, can't be an exit point
        else if ((grid.getMetricDistance(vertex->point.x, vertex->point.y) < minExitPointDistance)
                 && (num_source_cells_with_classification(vertex->point, SKELETON_CELL_FRONTIER, grid) == 0)) {
            continue;
        }
        // At this point, any cell that is a dead end is valid. The other filter have tossed out the bad ones.
        // Always trust the validity of dead ends, the filtering of which are good or bad has already happened
        // elsewhere
        else if (utils::contains(grid.getDeadEnds(), vertex->point)) {
            exitPoints.push_back(vertex);
        }
    }

    return exitPoints;
}


void SkeletonPruner::findReducedExitPoints(VoronoiSkeletonGrid& grid, const SkeletonGraph& revg)
{
    reducedExitPoints.clear();

    for (auto vertex : revg.getVertices()) {
        // If one adjacent and the vertex hasn't been pruned by other processes
        // then it is an exit point in the updated graph
        if ((vertex->numAdjacent == 1)
            && (grid.getClassification(vertex->point.x, vertex->point.y) & SKELETON_CELL_REDUCED_SKELETON)) {
            reducedExitPoints.push_back(vertex->point);
        }
    }
}


bool is_valid_start_cell(const cell_t& point, const VoronoiSkeletonGrid& grid, float minDistance, const pose_t& pose)
{
    // If not a skeleton cell or too close to a wall, then definitely not a valid start
    if ((~grid.getClassification(point.x, point.y) & SKELETON_CELL_SKELETON)
        || (grid.getMetricDistance(point.x, point.y) < minDistance)) {
        return false;
    }

    // Otherwise if the pose is valid then the cell needs to be reachable via the skeleton to ensure the
    // robot always remains within the current skeleton when the LPM is cut into multiple connected components
    auto poseCell = utils::global_point_to_grid_cell_round(pose.toPoint(), grid);

    // If the pose isn't in free space or in the map, then start is valid
    if (!grid.isCellInGrid(poseCell) || (~grid.getClassificationNoCheck(poseCell.x, poseCell.y) & SKELETON_CELL_FREE)) {
        return true;
    }

    auto path = find_path_along_skeleton(poseCell, point, SKELETON_CELL_SKELETON, grid);
    return path.result == VoronoiPathResult::success;
}


void prune_clumps(const SkeletonGraph& revg, VoronoiSkeletonGrid& grid)
{
    for (auto vertex : revg.getVertices()) {
        if (is_cell_nub(vertex->point, grid)) {
            grid.removeClassification(vertex->point.x, vertex->point.y, SKELETON_CELL_REDUCED_SKELETON);
        }
    }
}


bool is_cell_nub(cell_t cell, const VoronoiSkeletonGrid& grid, int maxLength)
{
    auto trace = trace_voronoi_graph(cell, grid, 2 * maxLength);
    int length = 0;
    int numDeadEnds = 0;
    int numJunctions = 0;
    for (auto& t : trace.traces) {
        length += t.points.empty() ? 0 : t.points.size() - 1;   // don't include the cell itself in the length

        if ((t.endCause == TRACE_DEAD_END) || (t.endCause == TRACE_FRONTIER)) {
            ++numDeadEnds;
        } else if (t.endCause == TRACE_JUNCTION) {
            ++numJunctions;
        }
    }

    return (length <= maxLength) && (numJunctions <= 1) && (numDeadEnds >= 1);
}

}   // namespace hssh
}   // namespace vulcan
