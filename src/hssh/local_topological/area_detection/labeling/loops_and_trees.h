/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     loops_and_trees.h
* \author   Collin Johnson
*
* Declaration of label_loops_and_trees function.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_PARSING_LOOPS_AND_TREES_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_PARSING_LOOPS_AND_TREES_H

#include "utils/algorithm_ext.h"
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace vulcan
{
namespace hssh
{

class AreaGraph;
class AreaNode;

// Hold the hypothesis for each vertex, which makes it easy to mark the
struct LoopGraphVertex
{
    AreaNode* node;
};

struct LoopGraphEdge
{
    boost::default_color_type color;
    double length;
};

// Define the concrete template classes to be used with BGL
using LoopGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, LoopGraphVertex, LoopGraphEdge>;

/**
* An EdgePredicate filter for LoopGraphs to exclude any edges between nodes in the excluded set.
*/
template <class Graph>
struct NotExcludedEdge
{
    using ExcludeSet = std::vector<AreaNode*>;

    const ExcludeSet* excluded;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        AreaNode* sourceNode = (*graph)[boost::source(edge, *graph)].node;
        AreaNode* targetNode = (*graph)[boost::target(edge, *graph)].node;
        return !utils::contains(*excluded, sourceNode) || !utils::contains(*excluded, targetNode);
    }

    explicit NotExcludedEdge(const ExcludeSet* excludeSet = nullptr, Graph* graph = nullptr)
    : excluded(excludeSet)
    , graph(graph)
    {
    }
};

/**
* construct_loop_graph creates the LoopGraph from an AreaGraph.
*/
void construct_loop_graph(AreaGraph& areaGraph, LoopGraph& loopGraph);

/**
* label_loops_and_trees searches through the graph and labels each AreaNode as being either on a loop or tree.
*
* To find the loops, a DFS is used. If a back edge (an edge leading to an already visited node) is found, then all
* predecessors in the search tree until the destination of the back edge are labelled as Loop. All areas are initially
* marked as Trees.
*
* \param    graph           The graph to be marked
*/
void label_loops_and_trees(AreaGraph& graph);

/**
* label_loops_and_trees runs the loops-and-tree algorithm for an already-constructed LoopGraph.
*/
void label_loops_and_trees(LoopGraph& graph);

/**
* compute_node_distances computes the distance between all nodes in the AreaGraph.
*
* Johnson's all-pairs algorithm is used to find the distance between all AreaNodes in the graph.
*
* \param    graph           AreaGraph in which to mark the distances
* \param    loopGraph       LoopGraph representation of the AreaGraph
*/
void compute_node_distances(AreaGraph& graph, const LoopGraph& loopGraph);


} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_PARSING_LOOPS_AND_TREES_H
