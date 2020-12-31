/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     search.h
* \author   Collin Johnson
*
* Declaration of various search classes for a TopologicalGraph:
*
*   - AStarSearch : do a traditional A* search
*/

#ifndef PLANNER_GOAL_SEARCH_H
#define PLANNER_GOAL_SEARCH_H

#include "hssh/global_topological/graph.h"
#include "planner/goal/debug_info.h"
#include "graph/path.h"
#include "utils/object_pool.h"
#include "utils/binary_heap.h"
#include <vector>
#include <set>
#include <map>

namespace vulcan
{
namespace planner
{

/**
* AStarSearch supports searching for Paths through a Graph using the A* algorithm. The A* implementation
* uses an as-the-crow-flies heuristic. To perform the search, the following are required of the Vertex and
* Edge template parameters, in addition to the requirements from Graph:
*
*   Vertex:
*   - getCost()     : cost of visiting the node
*   - getPosition() : position of the node (x,y,z)
*
*   Edge:
*   - getCost()     : cost of traversing the edge
*/
class AStarSearch
{
public:

    /**
    * Constructor for AStarSearch.
    *
    * \param    graph           Graph to be searched
    */
    AStarSearch(const hssh::TopologicalGraph& graph);

    /**
    * search searches through the Graph for a Path from start to goal. The search uses the traditional A* algorithm
    * with an as-the-crow-flies distance heuristic.
    *
    * \param    start           Starting vertex for the path
    * \param    goal            Goal vertex for the path
    * \return   True if a path exists. False otherwise.
    */
    bool search(const hssh::TopologicalVertex& start, const hssh::TopologicalVertex& goal);

    /**
    * getPath retrieves the Path associated with the most recent successful search through the graph.
    */
    const graph::Path<hssh::TopologicalVertex>& getPath(void) const { return path; }

    /**
    * getDebugInfo retrieves the debugging info associated with the most recent search, successful or not.
    */
    const graph_search_debug_info_t& getDebugInfo(void) const { return debug; }

private:

    typedef hssh::TopologicalVertex Vertex;
    typedef hssh::TopologicalEdge   Edge;

    struct astar_node_t
    {
        const Vertex* vertex;
        astar_node_t* parent;
        double        costSoFar;
        double        costToGo;

        astar_node_t(void)
            : vertex(0)
            , parent(0)
            , costSoFar(0.0)
            , costToGo(0.0)
        {
        }
    };

    void createNodesForGraph    (void);
    void initializeSearch       (const Vertex& start, const Vertex& goal);
    bool performSearch          (void);
    void expandNode             (astar_node_t* node);
    void queueNeighborIfLessCost(astar_node_t* parent, const Edge& edge);
    void extractPathFromGoalNode(astar_node_t* goalNode);

    bool isGoalNode(astar_node_t* node) { return node && (*(node->vertex) == goal); }

    friend bool compareNodes(astar_node_t* const& lhs, astar_node_t* const& rhs);

    Vertex goal;

    utils::BinaryHeap<astar_node_t*> searchQueue;
    std::map<int, astar_node_t*>     vertexToNode;
    std::set<int>                    expanded;
    utils::ObjectPool<astar_node_t>  nodes;        // use the ObjectPool for automatic cleanup on destruction

    const hssh::TopologicalGraph& topoGraph;

    graph::Path<Vertex>       path;
    graph_search_debug_info_t debug;

};

}
}

#endif // PLANNER_GOAL_SEARCH_H
