/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     heat_map.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalTopoHeatMap.
 */

#include "hssh/local_topological/evaluation/heat_map.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_topological/area.h"
#include "utils/algorithm_ext.h"
#include <random>

namespace vulcan
{
namespace hssh
{

void add_path_to_stats(const LocalTopoRoute& path, HeatMapStatistics& stats);


LocalTopoHeatMap::LocalTopoHeatMap(const LocalTopoMap& groundTruth, const LocalTopoMap& labeled)
: groundTruthGraph_(groundTruth)
, labeledGraph_(labeled)
, groundTruthMap_(groundTruth)
, labeledMap_(labeled)
, haveGroundTruth_(true)
{
    // Associate each area in the ground truth map with all of its extent cells
    associateAreasWithCells(groundTruthMap_, cellToGroundTruthArea_);

    // Associate each area in the labeled map with all of its extent cells
    associateAreasWithCells(labeledMap_, cellToLabeledArea_);

    // Extract all free space cells
    // Don't use an LPM for creating the free cells to sample from because not all free cells are in extents and
    // the extent flood fill isn't perfect and also leaves some cells out
    for (auto& area : groundTruth) {
        std::transform(area->extent().begin(),
                       area->extent().end(),
                       std::back_inserter(freeCells_),
                       [&groundTruth](auto position) {
                           return utils::global_point_to_grid_cell_round(position, groundTruth.voronoiSkeleton());
                       });
    }

    // Any cells that appear twice are along a boundary. Remove all of these cells because they will result in
    // potential inconsistency between which area is associated with the point between the labeled and ground-truth
    // which will artificially make the statistics worse
    std::sort(freeCells_.begin(), freeCells_.end());
    CellVector toRemove;
    for (auto cellIt = freeCells_.begin(), endIt = freeCells_.end() - 1; cellIt != endIt; ++cellIt) {
        if (*cellIt == *(cellIt + 1)) {
            toRemove.push_back(*cellIt);
        }
    }

    // Also remove all cells that are not in both maps -- slight differences due to gateways can exist
    for (auto cell : freeCells_) {
        if ((cellToGroundTruthArea_.find(cell) == cellToGroundTruthArea_.end())
            || (cellToLabeledArea_.find(cell) == cellToLabeledArea_.end())) {
            toRemove.push_back(cell);
        }
    }

    for (auto& cell : toRemove) {
        utils::erase_remove(freeCells_, cell);
    }
}


LocalTopoHeatMap::LocalTopoHeatMap(const LocalTopoMap& labeled) : LocalTopoHeatMap(labeled, labeled)
{
    haveGroundTruth_ = false;

    // Creates an unnecessary copy of the graph, but eh, it's okay.
}


void LocalTopoHeatMap::generatePaths(int numPaths)
{
    std::random_device rd;
    std::uniform_int_distribution<int> rng(0, freeCells_.size() - 1);

    for (int n = 0; n < numPaths; ++n) {
        int startIndex = rng(rd);
        int finishIndex = rng(rd);

        auto startCell = freeCells_[startIndex];
        auto finishCell = freeCells_[finishIndex];

        assert(cellToLabeledArea_.find(startCell) != cellToLabeledArea_.end());
        assert(cellToLabeledArea_.find(finishCell) != cellToLabeledArea_.end());

        auto labeledStart = std::make_pair(utils::grid_point_to_global_point(startCell, labeledMap_.voronoiSkeleton()),
                                           cellToLabeledArea_[startCell]);
        auto labeledFinish =
          std::make_pair(utils::grid_point_to_global_point(finishCell, labeledMap_.voronoiSkeleton()),
                         cellToLabeledArea_[finishCell]);

        auto labeledPath = labeledGraph_.findPath(labeledStart, labeledFinish);

        // If there's isn't ground-truth, always add the labeled path
        if (!haveGroundTruth_) {
            if (!labeledPath.empty()) {
                add_path_to_stats(labeledPath, labeledStats_);
            }
        } else {
            assert(cellToGroundTruthArea_.find(startCell) != cellToGroundTruthArea_.end());
            assert(cellToGroundTruthArea_.find(finishCell) != cellToGroundTruthArea_.end());
            auto groundTruthStart =
              std::make_pair(utils::grid_point_to_global_point(startCell, groundTruthMap_.voronoiSkeleton()),
                             cellToGroundTruthArea_[startCell]);
            auto groundTruthFinish =
              std::make_pair(utils::grid_point_to_global_point(finishCell, groundTruthMap_.voronoiSkeleton()),
                             cellToGroundTruthArea_[finishCell]);

            auto truthPath = groundTruthGraph_.findPath(groundTruthStart, groundTruthFinish);

            // Only add both paths if a path was found to ensure the stats that are computed are for the same
            // paths through the map
            if (!truthPath.empty() && !labeledPath.empty()) {
                add_path_to_stats(labeledPath, labeledStats_);
                add_path_to_stats(truthPath, groundTruthStats_);
            }
        }
    }
}


void LocalTopoHeatMap::associateAreasWithCells(const LocalTopoMap& map, ExtentToAreaMap& cellsToAreas)
{
    for (auto& area : map) {
        for (auto pos : area->extent()) {
            auto cell = utils::global_point_to_grid_cell_round(pos, map.voronoiSkeleton());
            cellsToAreas[cell] = area->id();
        }
    }
}


void add_path_to_stats(const LocalTopoRoute& path, HeatMapStatistics& stats)
{
    stats.paths.push_back(path);

    for (const LocalTopoRouteVisit& visit : path) {
        ++stats.areaVisitCount[visit.area().id()];
    }
}

}   // namespace hssh
}   // namespace vulcan
