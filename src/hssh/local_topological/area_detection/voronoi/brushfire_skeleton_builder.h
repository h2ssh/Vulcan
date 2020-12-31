/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     brushfire_skeleton_builder.h
 * \author   Collin Johnson
 *
 * Declaration of BrushfireSkeletonBuilder.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_BRUSHFIRE_SKELETON_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_BRUSHFIRE_SKELETON_BUILDER_H

#include "hssh/local_topological/area_detection/voronoi/skeleton_builder.h"
#include "utils/bucketed_binary_heap.h"
#include <queue>
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{

const std::string BRUSHFIRE_SKELETON_BUILDER_TYPE("brushfire");

/**
 * BrushfireSkeletonBuilder uses a brushfire algorithm to build the VoronoiSkeletonGrid. The algorithm works as follows:
 *
 * To initialize the brushfire, each FREE cell in the LPM that is bordered by an OCCUPIED cell is added
 * to the search queue. The main loop of the algorithm dequeues the first element in the search queue. This distance
 * of this cell is marked in the place grid. The cell is classified with the following criteria:
 *
 *   - FRONTIER
 *       0) The cell is FREE in the occupancy grid and an adjacent cell is UNKNOWN.
 *
 *   - FREE
 *       0) The cell is FREE or DYNAMIC in the occupancy grid and surrounded by FREE, DYNAMIC, OCCUPIED, or UNKNOWN
 * cells.
 *
 *   - OCCUPIED
 *       0) The cell is OCCUPIED in the occupancy grid.
 *
 *   - UNKNOWN
 *       0) This classification is the default for all cells in the place grid.
 *
 *   - SKELETON
 *       0) If the distance of the cell is >= to the distance of all adjacent cells and distance < coastal distance.
 *       2) If distance == coastal distance.
 */
class BrushfireSkeletonBuilder : public SkeletonBuilder
{
public:
    /**
     * Constructor for BrushfireSkeletonBuilder.
     */
    BrushfireSkeletonBuilder(const skeleton_builder_params_t& params);

    virtual ~BrushfireSkeletonBuilder(void);

private:
    using CellDistPair = std::pair<cell_t, VoronoiDist>;

    struct brushfire_node_t
    {
        brushfire_node_t(cell_t cell = cell_t(0, 0), VoronoiDist distance = 255, cell_t origin = cell_t(0, 0))
        : cell(cell)
        , distance(distance)
        , parent(origin)
        , origin(origin)
        {
        }

        brushfire_node_t(cell_t cell, VoronoiDist distance, cell_t parent, cell_t origin)
        : cell(cell)
        , distance(distance)
        , parent(parent)
        , origin(origin)
        {
        }

        VoronoiDist getPriority(void) const { return distance; }

        bool operator<(const brushfire_node_t& rhs) const { return distance < rhs.distance; }

        cell_t cell;
        VoronoiDist distance;

        cell_t parent;
        cell_t origin;
    };

    utils::BucketedBinaryHeap<brushfire_node_t, VoronoiDist> searchQueue;

    std::vector<CellVector> cellAnchors;
    std::vector<CellDistPair> unprunedSkeletonCells;

    int64_t numUpdates;
    int64_t sumInitializeTimeUs;
    int64_t sumEnqueueTimeUs;
    int64_t sumBrushfireTimeUs;
    int64_t sumPruneTimeUs;

    VoronoiDist coastalDistanceInCells;

    friend bool compareNodes(const BrushfireSkeletonBuilder::brushfire_node_t& lhs,
                             const BrushfireSkeletonBuilder::brushfire_node_t& rhs);

    /**
     * extractSkeleton organizes the computation for running the brushfire algorithm.
     */
    void extractSkeleton(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid, IslandDetector& islands);

    // Steps for the brushfire algorithm
    void initializeSearch(VoronoiSkeletonGrid& grid);
    void enqueueObstacleBoundaries(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid, IslandDetector& islands);
    void addObstacleCell(cell_t cell, VoronoiSkeletonGrid& grid);

    void runBrushfire(VoronoiSkeletonGrid& grid);
    void expandCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid);
    void processFreeCellNeighbor(const brushfire_node_t& node, cell_t neighbor, VoronoiSkeletonGrid& grid);
    void enqueueFreeCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid);
    void handleCollisionCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid);
    void addSkeletonCell(cell_t cell, cell_t origin, VoronoiSkeletonGrid& grid);

    void pruneSkeleton(VoronoiSkeletonGrid& grid);

    void setCoastalDistance(float cellScale);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_BRUSHFIRE_SKELETON_BUILDER_H
