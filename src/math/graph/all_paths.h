/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     all_paths.h
* \author   Collin Johnson
*
* Declaration of AllPaths graph reducer.
*/

#ifndef MATH_GRAPH_ALL_PATHS_H
#define MATH_GRAPH_ALL_PATHS_H

#include "core/point.h"
#include "math/graph/graph.h"
#include "utils/object_pool.h"
#include <map>
#include <queue>
#include <set>

namespace vulcan
{
namespace math
{

typedef Graph<uint16_t, 8> PathGraph;
typedef graph_vertex_t<uint16_t, 8> path_vertex_t;

/**
* AllPaths is an algorithm that extracts all paths that connect a set of vertices in a graph. Formally:
* Given a graph G(V,E) and a set of vertices S that is a subset of V, all-paths finds the subgraph of
* G that contains all acyclic paths connecting the vertices in S.
*/
class AllPaths
{
public:

    /**
    * Constructor for AllPaths.
    *
    * \param    useCachedGraph      Flag indicating if the returned graph should use vertices that are cached in AllPaths
    *                               meaning the returned subgraph memory is owned by AllPaths rather than the callee
    */
    AllPaths(bool useCachedGraph);

    /**
    * extractAllPathsSubgraph extracts the subgraph containing all paths between the provided vertices
    * in the graph.
    *
    * \param    graph           Graph from which the subgraph will be extracted
    * \param    vertices        Vertices to be connected in the subgraph
    * \return   Subgraph of the provided graph that contains all paths between the input vertices.
    */
    PathGraph extractAllPathsSubgraph(const PathGraph& graph, const std::vector<path_vertex_t*>& vertices);

private:

    static const int MAX_PARENTS = 8;

    struct all_paths_node_t
    {
        all_paths_node_t* parents[MAX_PARENTS];
        uint8_t           numParents;

        path_vertex_t* source;
        path_vertex_t* vertex;
        path_vertex_t* subgraphVertex;

        bool pathsAdded;
    };

    void initializeSearch(const std::vector<path_vertex_t*>& vertices);
    void runSearch(const PathGraph& graph, PathGraph& subgraph);

    void expandNode(all_paths_node_t* node, PathGraph& subgraph);

    bool isParentNode(all_paths_node_t* node, path_vertex_t* vertex);
    void handleActiveNodeEdge(all_paths_node_t* parent, all_paths_node_t* child, PathGraph& subgraph);
    void handleNewChildVertex(all_paths_node_t* parent, path_vertex_t* childVertex);

    void addPathToSubgraph(all_paths_node_t* node, PathGraph& subgraph);
    void addEdgeBetweenNodes(all_paths_node_t* leftNode, all_paths_node_t* rightNode, PathGraph& subgraph);

    all_paths_node_t* newNode(path_vertex_t* source, path_vertex_t* vertex, all_paths_node_t* parent);
    path_vertex_t*    newSubgraphVertex(path_vertex_t* graphVertex);

    std::queue<all_paths_node_t*>               nodeQueue;
    std::map<path_vertex_t*, all_paths_node_t*> activeNodes;
    std::set<path_vertex_t*>                    expandedNodes;

    utils::ObjectPool<all_paths_node_t> pathNodePool;
    utils::ObjectPool<path_vertex_t>    vertexPool;

    bool useVertexPool;
};

}
}

#endif // MATH_GRAPH_ALL_PATHS_H
