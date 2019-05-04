/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_graph_extractor.h
* \author   Collin Johnson
*
* Declaration of SkeletonGraphExtractor.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_EXTRACTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_EXTRACTOR_H

#include <map>
#include <queue>
#include <vector>
#include <set>
#include <utils/object_pool.h>
#include <hssh/local_topological/area_detection/voronoi/skeleton_graph.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;

/**
* SkeletonGraphExtractor parses through the VoronoiSkeletonGrid and converts the discretized cells
* into a true graph data structure to which graph algorithms can be applied.
*
* SkeletonGraphExtractor is meant to be used only in conjunction with the SkeletonPruner.
* The two classes are very much entwined. The SkeletonGraph structure isn't safe to use
* outside of the limited context due to the memory sharing that occurs within it. Rest
* of the system is abstracted behind the SkeletonPruner though, so all should be fine.
*
* If it comes to pass that the SkeletonGraph does need to exist outside the Pruner,
* then a deep copy of the graph will have to be made so that memory problems don't
* force the explosion of anything that attempts to touch it.
*/
class SkeletonGraphExtractor
{
public:

    SkeletonGraphExtractor(float minVertexDistance = 0.2999);
    ~SkeletonGraphExtractor(void);

    /**
    * extractGraph extracts the Graph from the current VoronoiSkeletonGrid. The graph extraction
    * extracts the continuous graph connected to the starting point. As such, the starting
    * point should be close to the current robot position, otherwise the graph might
    * not be connected to the skeleton closest to the robot.
    *
    * \param    startCells              Initial skeleton cells from which the search grows
    * \param    grid                    Grid containing the cell-based skeleton
    */
    SkeletonGraph extractGraph(const std::set<cell_t>& startCells, VoronoiSkeletonGrid& grid);

private:

    // Graph extractor cannot be copied and it doesn't make sense to do so
    SkeletonGraphExtractor(const SkeletonGraphExtractor& e) = delete;
    void operator=        (const SkeletonGraphExtractor& e) = delete;

    void initializeExtraction(void);
    void runExtraction(VoronoiSkeletonGrid& grid);
    void expandVertex(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid);
    void findAdjacentVertices(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid);
    bool considerVertex(skeleton_graph_vertex_t*    parent,
                        cell_t                      pointToConsider,
                        const Point<int16_t>& vertexDirection,
                        VoronoiSkeletonGrid&        grid);
    bool addEdge(skeleton_graph_vertex_t* parent, skeleton_graph_vertex_t* child);

    // Abstract away the vertex pool handling
    skeleton_graph_vertex_t* newSkeletonVertex(const Point<uint16_t>& point);

    std::map<cell_t, skeleton_graph_vertex_t*> activeVertices;
    std::queue<skeleton_graph_vertex_t*>  vertexQueue;

    int numEdges;
    int numVertices;
    
    float minVertexDistance_;

    utils::ObjectPool<skeleton_graph_vertex_t> vertexPool;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_EXTRACTOR_H
