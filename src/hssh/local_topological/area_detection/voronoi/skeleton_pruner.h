/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_pruner.h
* \author   Collin Johnson
*
* Declaration of SkeletonPruner for removing unnecessary branches of the skeleton.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_PRUNER_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_PRUNER_H

#include "hssh/local_topological/params.h"
#include "hssh/local_topological/area_detection/voronoi/skeleton_graph_extractor.h"
#include "math/graph/connected_components.h"
#include <memory>

namespace vulcan
{
struct pose_t;
namespace hssh
{

class SkeletonGraphReducer;
class VoronoiSkeletonGrid;

/**
* SkeletonPruner is responsible for taking the extended Voronoi graph built by a skeletonizing algorithm and pruning
* the superfluous vertices and edges to create a reduced extended Voronoi graph. The pruning process has three steps:
*
* 0) Extract the underlying graph structure from the cell-based skeleton. Build a traditional edges-vertices representation.
* 1) Reduce the graph using a SkeletonGraphReducer. Either minimum-spanning tree or all-paths at the moment.
* 2) Rasterize the reduced graph back into the cell grid.
*/
class SkeletonPruner
{
public:

    /**
    * Constructor for SkeletonPruner.
    */
    SkeletonPruner(const skeleton_pruner_params_t& params);

    /**
    * Destructor for SkeletonPruner.
    */
    ~SkeletonPruner(void);

    /**
    * pruneSkeleton takes a skeleton embedded in a VoronoiSkeletonGrid and reduces it to a skeleton
    * of cells that links only the exit points of the skeleton. The branches that terminate
    * on non-exit points are removed.
    *
    * Cells that are part of the reduced extended Voronoi graph will be marked as SKELETON_CELL_REDUCED_SKELETON.
    *
    * \param    grid                Skeleton to be pruned
    * \param    pose                Current robot pose to help determine the valid subgraph
    * \param    pointsOfInterest    Additional points of interest beyond dead ends and exit points for keeping in
    *                               the reduced skeleton
    */
    void pruneSkeleton(VoronoiSkeletonGrid& grid,
                       const pose_t& pose,
                       const std::vector<Point<float>>& pointsOfInterest);

    /**
    * getExitPoints retrieves all exit points found in the reduced EVG.
    */
    std::vector<cell_t> getExitPoints(void) const { return reducedExitPoints; }

private:

    std::vector<skeleton_graph_vertex_t*> findExitVertices(const VoronoiSkeletonGrid& grid, const SkeletonGraph& evg);
    void findReducedExitPoints(VoronoiSkeletonGrid& grid, const SkeletonGraph& revg);

    SkeletonGraphExtractor                extractor;
    std::unique_ptr<SkeletonGraphReducer> reducer;
    math::ConnectedComponents             connected;

    std::vector<cell_t> reducedExitPoints;

    float minExitPointDistance;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_PRUNER_H
