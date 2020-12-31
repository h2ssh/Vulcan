/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     voronoi_utils.cpp
 * \author   Collin Johnson
 *
 * Definition of voronoi_direction() and best_anchor_boundary().
 */

#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "core/angle_functions.h"
#include "core/line.h"
#include "core/point.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "math/regression.h"
#include "utils/ray_tracing.h"
#include <iostream>
#include <queue>
#include <set>

// #define DEBUG_DIRECTION
// #define DEBUG_TRACE

#if defined DEBUG_DIRECTION || defined DEBUG_TRACE
    #include <iostream>
#endif

namespace vulcan
{
namespace hssh
{

const double kMaxTraceLength = 10.0;
const double kCloseAngleThreshold = M_PI / 6.0;

class CellMatchesMask
{
public:
    CellMatchesMask(uint8_t mask) : mask(mask) { }

    bool operator()(const VoronoiSkeletonGrid& grid, Point<int> cell)
    {
        return grid.getClassification(cell.x, cell.y) & mask;
    }

private:
    uint8_t mask;
};


inline bool is_direction_close_to_angle_hint(double angle, double angleHint, double threshold)
{
    return angle_diff_abs(angle, angleHint) < threshold;
}


template <class Op>
std::size_t four_or_eight_connectivity(const cell_t& cell,
                                       uint8_t classification,
                                       const VoronoiSkeletonGrid& grid,
                                       bool useFourWay,
                                       Op comp,
                                       NeighborArray& neighbors);

template <class Op>
std::size_t four_then_eight_connectivity(const cell_t& cell,
                                         uint8_t classification,
                                         const VoronoiSkeletonGrid& grid,
                                         Op comp,
                                         NeighborArray& neighbors);

int follow_branch(const cell_t& startCell,
                  const VoronoiSkeletonGrid& grid,
                  int maxTraceDistance,
                  int numJunctionsBeforeHalt,
                  uint8_t neighborMask,
                  std::set<cell_t>& visited,
                  std::vector<voronoi_trace_cells_t>& traces);
template <class Iter>
Iter extract_cells_along_edge(cell_t parent, cell_t child, const VoronoiSkeletonGrid& grid, Iter edgeOut);

voronoi_directions_t find_directions_from_trace(const voronoi_trace_t& trace, double angleHint);


////////////////////// Main functions ///////////////////////////////////////

std::vector<cell_t> extract_cells_with_mask(const VoronoiSkeletonGrid& grid, uint8_t mask, int step)
{
    std::vector<cell_t> cells;
    cells.reserve(std::distance(grid.beginSkeletonCells(), grid.endSkeletonCells()));
    for (std::size_t y = 0; y < grid.getHeightInCells(); y += step) {
        for (std::size_t x = 0; x < grid.getWidthInCells(); x += step) {
            if (grid.getClassification(x, y) & mask) {
                cells.emplace_back(x, y);
            }
        }
    }
    return cells;
}


SourceToCellsMap extract_source_cells(const VoronoiSkeletonGrid& grid, uint8_t skeletonMask)
{
    SourceToCellsMap sourceToCells;

    const auto& skeletonToSources = grid.getSkeletonSources();

    for (auto& cellToSources : skeletonToSources) {
        if (!(grid.getClassification(cellToSources.first.x, cellToSources.first.y) & skeletonMask)) {
            continue;
        }

        for (auto& source : cellToSources.second) {
            sourceToCells[source].push_back(cellToSources.first);
        }
    }

    return sourceToCells;
}


float cell_vector_perimeter(CellConstIter begin, CellConstIter end, float metersPerCell)
{
    return std::distance(begin, end) * metersPerCell;

    //     float perimeter = 0.0f;
    //
    //     for(auto cIt = begin; cIt != end; ++cIt)
    //     {
    //         // See how many sides are shared by the next and previous cells
    //         auto nextIt = cIt + 1;
    //         if(nextIt == end)
    //         {
    //             nextIt = cIt - 1;
    //         }
    //
    //         auto prevIt = cIt - 1;
    //         if(cIt == begin)
    //         {
    //             prevIt = end - 1;
    //         }
    //
    //         int numShared = 0;
    //
    //         // Four-way connected?
    //         if((cIt->x == nextIt->x) || (cIt->y == nextIt->y))
    //         {
    //             ++numShared;
    //         }
    //
    //         if((cIt->x == prevIt->x) || (cIt->y == prevIt->y))
    //         {
    //             ++numShared;
    //         }
    //
    //         // One or both eight-way neighbors means two sides are to the outside
    //         if(numShared < 2)
    //         {
    //             perimeter += metersPerCell + metersPerCell;
    //         }
    //         // Both four way means that just one side is to the outside
    //         else
    //         {
    //             perimeter += metersPerCell;
    //         }
    //     }
    //
    //     return perimeter;
}


int num_source_cells_with_classification(cell_t cell, uint8_t mask, const VoronoiSkeletonGrid& grid)
{
    return std::count_if(grid.beginSourceCells(cell), grid.endSourceCells(cell), [&](cell_t source) {
        return grid.getClassification(source.x, source.y) & mask;
    });
}


std::size_t neighbor_cells_with_classification(const cell_t& cell,
                                               uint8_t classification,
                                               const VoronoiSkeletonGrid& grid,
                                               cell_connectivity_t connectivity,
                                               NeighborArray& neighbors)
{
    switch (connectivity) {
    case FOUR_THEN_EIGHT_WAY:
        return four_then_eight_connectivity(cell, classification, grid, std::bit_and<uint8_t>(), neighbors);

    case FOUR_WAY:
    case EIGHT_WAY:
    default:
        return four_or_eight_connectivity(cell,
                                          classification,
                                          grid,
                                          connectivity == FOUR_WAY,
                                          std::bit_and<uint8_t>(),
                                          neighbors);
    }

    return 0;
}


std::size_t neighbor_cells_equal_classification(const cell_t& cell,
                                                uint8_t classification,
                                                const VoronoiSkeletonGrid& grid,
                                                cell_connectivity_t connectivity,
                                                NeighborArray& neighbors)
{
    switch (connectivity) {
    case FOUR_THEN_EIGHT_WAY:
        return four_then_eight_connectivity(cell, classification, grid, std::equal_to<uint8_t>(), neighbors);

    case FOUR_WAY:
    case EIGHT_WAY:
    default:
        return four_or_eight_connectivity(cell,
                                          classification,
                                          grid,
                                          connectivity == FOUR_WAY,
                                          std::equal_to<uint8_t>(),
                                          neighbors);
    }

    return 0;
}


int num_neighbor_cells_with_classification(const cell_t& cell,
                                           uint8_t classification,
                                           const VoronoiSkeletonGrid& grid,
                                           cell_connectivity_t connectivity)
{
    NeighborArray neighbors;
    return neighbor_cells_with_classification(cell, classification, grid, connectivity, neighbors);
}


cell_t trace_ray_to_cell(float angle, const Point<int>& start, uint8_t mask, const VoronoiSkeletonGrid& grid)
{
    return utils::trace_ray_until_condition<VoronoiSkeletonGrid>(
      start,
      angle,
      grid.getWidthInMeters() * grid.getHeightInMeters(),   // trace to the edge of the map
      grid,
      CellMatchesMask(mask));
}


std::vector<cell_t> trace_rays_in_range(const utils::ray_trace_range_t& range,
                                        const Point<int>& start,
                                        uint8_t mask,
                                        const VoronoiSkeletonGrid& grid)
{
    auto endpoints = utils::trace_range_until_condition<VoronoiSkeletonGrid>(
      start,
      range,
      grid.getWidthInMeters() * grid.getHeightInMeters(),   // trace as far as needed
      grid,
      CellMatchesMask(mask));
    std::vector<cell_t> roundedToCell(endpoints.size());
    std::copy(endpoints.begin(), endpoints.end(), roundedToCell.begin());
    return roundedToCell;
}


voronoi_trace_t trace_voronoi_graph(const cell_t& cell,
                                    const VoronoiSkeletonGrid& grid,
                                    int maxTraceLength,
                                    int numJunctionsBeforeHalt,
                                    bool followOnlyReduced)
{
    voronoi_trace_t trace;
    trace.skeletonCell = cell;

    uint8_t mask = followOnlyReduced ? SKELETON_CELL_REDUCED_SKELETON : SKELETON_CELL_SKELETON;
    std::set<cell_t> pointsVisited;
    NeighborArray skeletonNeighbors;
    std::size_t numNeighbors =
      neighbor_cells_with_classification(cell, mask, grid, FOUR_THEN_EIGHT_WAY, skeletonNeighbors);

    pointsVisited.insert(cell);

    for (std::size_t n = 0; n < numNeighbors; ++n) {
        // The first cell in the trace is the skeleton cell, so follow only for maxLength - 1, not maxLength
        follow_branch(skeletonNeighbors[n],
                      grid,
                      maxTraceLength - 1,
                      numJunctionsBeforeHalt,
                      mask,
                      pointsVisited,
                      trace.traces);
    }

    for (auto& t : trace.traces) {
        t.points.insert(t.points.begin(), cell);
    }

#ifdef DEBUG_TRACE
    std::cout << "DEBUG::trace_voronoi_graph: Cell:" << cell << " Halt junctions:" << numJunctionsBeforeHalt
              << " Num traces:" << trace.traces.size() << ":\n";
    for (auto& t : trace.traces) {
        std::cout << "Trace: Size:" << t.points.size() << " End:" << t.points.back() << '\n';
    }
#endif

    if (trace.traces.empty()) {
        voronoi_trace_cells_t dummyTrace;
        dummyTrace.points.push_back(cell);
        dummyTrace.endCause = TRACE_DEAD_END;
        trace.traces.emplace_back(std::move(dummyTrace));
    }

    return trace;
}


int extract_edge_from_skeleton(cell_t start, const VoronoiSkeletonGrid& grid, CellVector& edgeOut)
{
    NeighborArray neighbors;
    int numNeighbors =
      neighbor_cells_with_classification(start, SKELETON_CELL_REDUCED_SKELETON, grid, FOUR_THEN_EIGHT_WAY, neighbors);

    // This function doesn't deal with junction points!
    if ((numNeighbors > 2) || (numNeighbors == 0)) {
        return 0;
    }

    auto edgeStart = std::back_inserter(edgeOut);

    // Search one edge and push the cells in order search
    auto edgeEnd = extract_cells_along_edge(start, neighbors[0], grid, edgeStart);

    // Reverse the order so they go from furthest to closest
    std::reverse(edgeOut.begin(), edgeOut.end());

    // Add the start cell
    *edgeEnd++ = start;

    // Append the remaining cells in the other direction along the cell
    if (numNeighbors == 2) {
        edgeEnd = extract_cells_along_edge(start, neighbors[1], grid, edgeEnd);
    }

    return edgeOut.size();
}


voronoi_directions_t voronoi_direction(const cell_t& skeletonCell, const VoronoiSkeletonGrid& grid, double angleHint)
{
    return voronoi_direction(trace_voronoi_graph(skeletonCell, grid, kMaxTraceLength * grid.cellsPerMeter(), 1),
                             angleHint);
}


voronoi_directions_t voronoi_direction(const voronoi_trace_t& trace, double angleHint)
{
    return find_directions_from_trace(trace, angleHint);
}


template <class Op>
std::size_t four_or_eight_connectivity(const cell_t& cell,
                                       uint8_t classification,
                                       const VoronoiSkeletonGrid& grid,
                                       bool useFourWay,
                                       Op comp,
                                       NeighborArray& neighbors)
{
    std::size_t numNeighbors = 0;
    int width = grid.getWidthInCells();
    int height = grid.getHeightInCells();

    if ((cell.x > 0) && comp(grid.getClassification(cell.x - 1, cell.y), classification)) {
        neighbors[numNeighbors++] = cell_t(cell.x - 1, cell.y);
    }

    if ((cell.x + 1 < width) && comp(grid.getClassification(cell.x + 1, cell.y), classification)) {
        neighbors[numNeighbors++] = cell_t(cell.x + 1, cell.y);
    }

    if ((cell.y > 0) && comp(grid.getClassification(cell.x, cell.y - 1), classification)) {
        neighbors[numNeighbors++] = cell_t(cell.x, cell.y - 1);
    }

    if ((cell.y + 1 < height) && comp(grid.getClassification(cell.x, cell.y + 1), classification)) {
        neighbors[numNeighbors++] = cell_t(cell.x, cell.y + 1);
    }

    if (!useFourWay) {
        if ((cell.x > 0) && (cell.y > 0) && comp(grid.getClassification(cell.x - 1, cell.y - 1), classification)) {
            neighbors[numNeighbors++] = cell_t(cell.x - 1, cell.y - 1);
        }

        if ((cell.x + 1 < width) && (cell.y > 0)
            && comp(grid.getClassification(cell.x + 1, cell.y - 1), classification)) {
            neighbors[numNeighbors++] = cell_t(cell.x + 1, cell.y - 1);
        }

        if ((cell.y + 1 < height) && (cell.x > 0)
            && comp(grid.getClassification(cell.x - 1, cell.y + 1), classification)) {
            neighbors[numNeighbors++] = cell_t(cell.x - 1, cell.y + 1);
        }

        if ((cell.y + 1 < height) && (cell.x + 1 < width)
            && comp(grid.getClassification(cell.x + 1, cell.y + 1), classification)) {
            neighbors[numNeighbors++] = cell_t(cell.x + 1, cell.y + 1);
        }
    }

    return numNeighbors;
}


template <class Op>
std::size_t four_then_eight_connectivity(const cell_t& cell,
                                         uint8_t classification,
                                         const VoronoiSkeletonGrid& grid,
                                         Op comp,
                                         NeighborArray& neighbors)
{
    /*
     * The adjacent vertices looks first for four-way connectivity amongst the cells. If no cells
     * are found along a given axis, the diagonals are checked. The reason for this approach is to
     * keep multiple paths from being extracted between cells. The main case is this:
     *
     *            C
     *          A B D
     *
     * If eight-way connectivity is used, then A-B-D is added and A-C-D is added. This fact is just
     * an artifact of my extraction method, but nonetheless is an issue, so the four-then-eight
     * connectivity handles this situation, A-B-D, and A-B-C are created appropriately.
     */

    std::size_t numNeighbors = 0;

    Point<int16_t> vertexDirection;
    cell_t vertexConsidered;

    int16_t xDir[2] = {-1, 1};
    int16_t yDir[2] = {-1, 1};

    int width = grid.getWidthInCells();
    int height = grid.getHeightInCells();

    vertexDirection.y = 0;
    vertexConsidered.y = cell.y;
    for (int n = 2; --n >= 0;) {
        vertexDirection.x = xDir[n];
        vertexConsidered.x = cell.x + xDir[n];

        if ((vertexConsidered.x >= 0) && (vertexConsidered.x < width)
            && comp(grid.getClassification(vertexConsidered.x, vertexConsidered.y), classification)) {
            xDir[n] = 0;
            neighbors[numNeighbors++] = vertexConsidered;
        }
    }

    vertexDirection.x = 0;
    vertexConsidered.x = cell.x;
    for (int n = 2; --n >= 0;) {
        vertexDirection.y = yDir[n];
        vertexConsidered.y = cell.y + yDir[n];

        if ((vertexConsidered.y >= 0) && (vertexConsidered.y < height)
            && comp(grid.getClassification(vertexConsidered.x, vertexConsidered.y), classification)) {
            yDir[n] = 0;
            neighbors[numNeighbors++] = vertexConsidered;
        }
    }

    for (int n = 2; --n >= 0;) {
        for (int m = 2; --m >= 0;) {
            if ((yDir[m] != 0) && (xDir[n] != 0)) {
                vertexDirection.x = xDir[n];
                vertexDirection.y = yDir[m];

                vertexConsidered = cell + vertexDirection;

                if (grid.isCellInGrid(vertexConsidered)
                    && comp(grid.getClassification(vertexConsidered.x, vertexConsidered.y), classification)) {
                    neighbors[numNeighbors++] = vertexConsidered;
                }
            }
        }
    }

    return numNeighbors;
}


int follow_branch(const cell_t& startCell,
                  const VoronoiSkeletonGrid& grid,
                  int maxTraceDistance,
                  int numJunctionsBeforeHalt,
                  uint8_t neighborMask,
                  std::set<cell_t>& visited,
                  std::vector<voronoi_trace_cells_t>& traces)
{
    // Following a branch is a simple DFS where the search ends as soon as a junction is encountered.
    // As a result, no queue needs to be maintained, just the next cell to be expanded will moving
    // along the branch.

    voronoi_trace_cells_t branchTrace;
    std::size_t branchStart = traces.size();
    NeighborArray skeletonNeighbors;
    std::size_t numNeighbors = 0;

    int numPoints = 0;
    bool haveNewNeighbor = true;

    // If there's no tracing to do be done, create a single cell trace to satisfy the function requirements
    if (maxTraceDistance == 0) {
        branchTrace.points.push_back(startCell);
        branchTrace.endCause = TRACE_DISTANCE;
        traces.push_back(std::move(branchTrace));
        return 1;
    }

    cell_t currentCell = startCell;

    // If no neighbor added, then reached an end of the skeleton, so jump out of the search
    while ((numPoints < maxTraceDistance) && haveNewNeighbor) {
        visited.insert(currentCell);
        branchTrace.points.push_back(currentCell);

        haveNewNeighbor = false;
        ++numPoints;

        numNeighbors =
          neighbor_cells_with_classification(currentCell, neighborMask, grid, FOUR_THEN_EIGHT_WAY, skeletonNeighbors);

        // only consider cells with fewer than three neighbors. If at least three neighbors, then some sort of Voronoi
        // junction encountered
        if (numNeighbors > 2) {
            break;
        }

        for (std::size_t n = 0; n < numNeighbors; ++n) {
            if (visited.find(skeletonNeighbors[n]) == visited.end()) {
                currentCell = skeletonNeighbors[n];
                haveNewNeighbor = true;
            }
        }
    }

    // If a junction was encountered and allowed to follow the branches, then do so. For each unvisited neighbor, follow
    // that branch. After the process finishes, prepend the trace for this branch to the children branches to create the
    // continuous line of cells for the DFS along that particular branch of the skeleton
    if ((numNeighbors > 2) && (numJunctionsBeforeHalt > 0) && (numPoints < maxTraceDistance)) {
        for (std::size_t n = 0; n < numNeighbors; ++n) {
            if (visited.find(skeletonNeighbors[n]) == visited.end()) {
                follow_branch(skeletonNeighbors[n],
                              grid,
                              maxTraceDistance - branchTrace.points.size(),
                              numJunctionsBeforeHalt - 1,
                              neighborMask,
                              visited,
                              traces);
            }
        }

        for (std::size_t n = branchStart; n < traces.size(); ++n) {
            traces[n].points.insert(traces[n].points.begin(), branchTrace.points.begin(), branchTrace.points.end());
        }
    }
    // At the bottom of the recursion, so just push this trace into the collection
    else {
        branchTrace.endCause =
          (numNeighbors > 2) ? TRACE_JUNCTION : (numPoints == maxTraceDistance) ? TRACE_DISTANCE : TRACE_DEAD_END;
        traces.push_back(std::move(branchTrace));
    }

    return branchTrace.points.size();
}


template <class Iter>
Iter extract_cells_along_edge(cell_t parent, cell_t child, const VoronoiSkeletonGrid& grid, Iter edgeOut)
{
    NeighborArray neighbors;

    // Simple DFS along the edge of the voronoi skeleton. Keep searching until there aren't exactly two neighbors
    // for the current child, i.e. at a dead end or a junction
    while (true) {
        *edgeOut++ = child;

        int numNeighbors = neighbor_cells_with_classification(child,
                                                              SKELETON_CELL_REDUCED_SKELETON,
                                                              grid,
                                                              FOUR_THEN_EIGHT_WAY,
                                                              neighbors);

        // Once a junction or dead end is reached, then finished with the search
        if (numNeighbors != 2) {
            break;
        }

        cell_t oldChild = child;
        // Search in the opposite direction from the parent
        child = (neighbors[0] == parent) ? neighbors[1] : neighbors[0];
        parent = oldChild;
    }

    return edgeOut;
}


voronoi_directions_t find_directions_from_trace(const voronoi_trace_t& trace, double angleHint)
{
    assert(!trace.traces.empty());

    double otherHint = angle_sum(angleHint, M_PI);
    double minHintDist = 1000.0;
    double minOtherDist = 1000.0;

    voronoi_directions_t directions = {angleHint, otherHint, minHintDist, minOtherDist};

    for (auto& t : trace.traces) {
        if (t.points.size() < 2) {
            continue;
        }

        auto traceLine = math::total_least_squares(t.points.begin(), t.points.end());
        double traceDirection = angle_to_point(traceLine.a, traceLine.b);

        if (angle_diff_abs(traceDirection, angleHint) < minHintDist) {
            directions.left = traceDirection;
            directions.leftDist = t.points.size();
            minHintDist = angle_diff_abs(traceDirection, angleHint);
        }

        if (angle_diff_abs(traceDirection, otherHint) < minOtherDist) {
            directions.right = traceDirection;
            directions.rightDist = t.points.size();
            minOtherDist = angle_diff_abs(traceDirection, otherHint);
        }
    }

    if (!is_direction_close_to_angle_hint(directions.left, angleHint, kCloseAngleThreshold)) {
        directions.left = angleHint;
    }

    if (!is_direction_close_to_angle_hint(directions.right, otherHint, kCloseAngleThreshold)) {
        directions.right = otherHint;
    }

    if (angle_diff_abs(directions.left, directions.right) < M_PI / 3.0) {
        directions.right = angle_sum(directions.right, M_PI);
    }

    return directions;
}

}   // namespace hssh
}   // namespace vulcan
