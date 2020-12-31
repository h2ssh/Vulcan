/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     frontiers_set.cpp
 * \author   Collin Johnson
 *
 * Definition of FrontiersSet.
 */

#include "hssh/local_topological/area_detection/voronoi/frontiers_set.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/frontier.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>

namespace vulcan
{
namespace hssh
{

FrontiersSet::FrontiersSet(const VoronoiSkeletonGrid& grid, float farFromOccupiedDistance)
: farFromOccupiedDistance(farFromOccupiedDistance * grid.cellsPerMeter())
{
    const auto& frontierCells = grid.getFrontierCells();
    CellSet addedCells;
    addedCells.reserve(frontierCells.size());
    NeighborArray neighbors;

    for (auto& cell : frontierCells) {
        if (addedCells.find(cell) == addedCells.end()) {
            if (neighbor_cells_equal_classification(cell, SKELETON_CELL_OCCUPIED, grid, FOUR_THEN_EIGHT_WAY, neighbors)
                > 0) {
                frontiers.push_back(Frontier(frontiers.size()));
                growFrontier(cell, addedCells, frontiers.size() - 1, grid);
            }
        }
    }
}


Frontier FrontiersSet::getCellFrontier(cell_t frontierCell) const
{
    auto indexIt = cellToFrontierIndex.find(frontierCell);

    if (indexIt != cellToFrontierIndex.end()) {
        return frontiers[indexIt->second];
    } else {
        return Frontier(INVALID_FRONTIER);
    }
}


bool FrontiersSet::isFarFromOccupied(cell_t frontierCell) const
{
    auto indexIt = cellToFrontierIndex.find(frontierCell);

    // If not a frontier, it must be okay.
    if (indexIt == cellToFrontierIndex.end()) {
        return false;
    }

    const Frontier& frontier = frontiers[indexIt->second];

    auto cellPosIt = std::find(frontier.cells.begin(), frontier.cells.end(), frontierCell);

    assert(cellPosIt
           != frontier.cells
                .end());   // the cell must be with this frontier, else some error happened when creating the frontiers

    return farFromOccupiedDistance
      < std::min(std::distance(frontier.cells.begin(), cellPosIt), std::distance(cellPosIt, frontier.cells.end()));
}


void FrontiersSet::growFrontier(cell_t start,
                                CellSet& addedCells,
                                std::size_t frontierIndex,
                                const VoronoiSkeletonGrid& grid)
{
    // Use a simple BFS to grow out the frontier. It isn't 100% accurate on tracing the boundary in the event of a fork
    // in the frontier, but it should be pretty close.
    Frontier& frontier = frontiers[frontierIndex];
    NeighborArray neighbors;

    std::queue<cell_t> cellQueue;
    cellQueue.push(start);
    addedCells.insert(start);

    while (!cellQueue.empty()) {
        auto& currentCell = cellQueue.front();
        std::size_t numNeighbors =
          neighbor_cells_with_classification(currentCell, SKELETON_CELL_FRONTIER, grid, FOUR_THEN_EIGHT_WAY, neighbors);

        frontier.cells.push_back(currentCell);
        cellToFrontierIndex.insert(std::make_pair(currentCell, frontierIndex));

        // Only add neighbors that haven't been visited already
        for (std::size_t n = 0; n < numNeighbors; ++n) {
            cell_t& neighbor = neighbors[n];

            if (addedCells.find(neighbor) == addedCells.end()) {
                cellQueue.push(neighbor);
                addedCells.insert(neighbor);
            }
        }

        cellQueue.pop();
    }
}

}   // namespace hssh
}   // namespace vulcan
