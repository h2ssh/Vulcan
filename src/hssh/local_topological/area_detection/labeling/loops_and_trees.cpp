/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     loops_and_trees.cpp
 * \author   Collin Johnson
 *
 * Definition of label_loops_and_trees function.
 */

#include "hssh/local_topological/area_detection/labeling/loops_and_trees.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <iostream>
#include <stack>

#define DEBUG_LOOPS
// #define DEBUG_CONNECTED

namespace vulcan
{
namespace hssh
{

using namespace boost;

using Vertex = graph_traits<LoopGraph>::vertex_descriptor;
using Edge = graph_traits<LoopGraph>::edge_descriptor;

// Define property map for storing the distances between nodes
using DistProperty = exterior_vertex_property<LoopGraph, double>;
using DistVec = DistProperty::container_type;
using DistMatrix = DistProperty::matrix_type;
using DistMap = DistProperty::matrix_map_type;

/*
 * LoopLabellingVisitor is a DFSVisitor for BGL.
 */
struct LoopLabellingVisitor : public default_bfs_visitor
{
    LoopLabellingVisitor(std::vector<Vertex>& pred, std::vector<Vertex>& source, std::vector<Vertex>& target)
    : predecessors(pred)
    , sourcePath(source)
    , targetPath(target)
    {
    }

    // DFSVisitor concept interface
    void initialize_vertex(Vertex v, const LoopGraph& graph);
    void tree_edge(Edge e, const LoopGraph& graph);
    void non_tree_edge(Edge e, const LoopGraph& graph);

    std::vector<Vertex>& predecessors;
    std::vector<Vertex>& sourcePath;
    std::vector<Vertex>& targetPath;
};


void construct_loop_graph(AreaGraph& areaGraph, LoopGraph& loopGraph)
{
    std::unordered_map<AreaNode*, Vertex> nodeToVertex;

    std::for_each(areaGraph.beginNodes(), areaGraph.endNodes(), [&](const auto& node) {
        auto v = add_vertex(loopGraph);
        nodeToVertex[node.get()] = v;
        loopGraph[v].node = node.get();
    });

    std::vector<std::pair<AreaNode*, AreaNode*>> addedEdges;

    std::for_each(areaGraph.beginEdges(), areaGraph.endEdges(), [&](const auto& edge) {
        auto endpoints = edge->getEndpoints();

        for (auto& added : addedEdges) {
            // Ignore previously added edges
            if ((added.first == endpoints[0].get()) && (added.second == endpoints[1].get())) {
                return;
            }
        }

        auto e = add_edge(nodeToVertex[endpoints[0].get()], nodeToVertex[endpoints[1].get()], loopGraph);
        loopGraph[e.first].length = edge->getLength();
        addedEdges.emplace_back(endpoints[0].get(), endpoints[1].get());
        addedEdges.emplace_back(endpoints[1].get(), endpoints[0].get());
    });
}


void label_loops_and_trees(AreaGraph& graph)
{
    LoopGraph loopGraph;
    construct_loop_graph(graph, loopGraph);
    label_loops_and_trees(loopGraph);
}


void label_loops_and_trees(LoopGraph& graph)
{
    std::vector<Vertex> predecessors(num_vertices(graph), num_vertices(graph));
    // storage for extracting the loops
    std::vector<Vertex> sourcePath;
    std::vector<Vertex> targetPath;
    LoopLabellingVisitor vis(predecessors, sourcePath, targetPath);
    predecessors[0] = 0;   // point root to itself

    breadth_first_search(graph, vertex(0, graph), visitor(vis));
    //     undirected_dfs(graph, visitor(vis).edge_color_map(get(&LoopGraphEdge::color, graph)));
}


void compute_node_distances(AreaGraph& graph, const LoopGraph& loopGraph)
{
    DistMatrix distances(graph.sizeNodes());
    DistMap distMap(distances, loopGraph);
    johnson_all_pairs_shortest_paths(loopGraph, distMap, weight_map(get(&LoopGraphEdge::length, loopGraph)));

    for (std::size_t n = 0, end = graph.sizeNodes(); n < end; ++n) {
        AreaNode* startNode = loopGraph[n].node;

        for (std::size_t m = n + 1; m < end; ++m) {
            AreaNode* endNode = loopGraph[m].node;
            graph.setNodeDistance(startNode, endNode, distances[n][m]);
        }
    }
}


void LoopLabellingVisitor::initialize_vertex(Vertex v, const LoopGraph& graph)
{
    // For each vertex, ensure it doesn't have any loops specified
    graph[v].node->setLoop(false);
}


void LoopLabellingVisitor::tree_edge(Edge e, const LoopGraph& graph)
{
    predecessors[target(e, graph)] = source(e, graph);
}


void LoopLabellingVisitor::non_tree_edge(Edge e, const LoopGraph& graph)
{
    // A back edge was found to the target of the edge
    // Go through the predecessors until target the is found.
    // Each node along the way back should be marked as part of a Loop

    auto targetVertex = target(e, graph);

    // Ignore this case. It arises from the BFS just looking at all outedges in the undirected graph, so it
    // sees back to the parent in the search tree when expanded.
    if (predecessors[source(e, graph)] == targetVertex) {
        return;
    }

    auto prevVertex = predecessors[targetVertex];
    auto vertex = targetVertex;

    // Extract the source and target paths
    targetPath.clear();
    while (prevVertex != vertex) {
        targetPath.push_back(vertex);
        prevVertex = vertex;
        vertex = predecessors[vertex];
    }

    vertex = source(e, graph);
    prevVertex = predecessors[vertex];
    sourcePath.clear();
    while (prevVertex != vertex) {
        sourcePath.push_back(vertex);
        prevVertex = vertex;
        vertex = predecessors[vertex];
    }

    // Find the first shared vertex between the two
    auto targetIt = std::find_first_of(targetPath.begin(), targetPath.end(), sourcePath.begin(), sourcePath.end());
    assert(targetIt != targetPath.end());
    auto sourceIt = std::find(sourcePath.begin(), sourcePath.end(), *targetIt);
    assert(sourceIt != sourcePath.end());

    // Turn begin, targetIt into a valid half-open range for a for-loop
    std::size_t targetEnd = std::distance(targetPath.begin(), targetIt) + 1;
    std::size_t sourceEnd = std::distance(sourcePath.begin(), sourceIt) + 1;

    double loopDist = 0.0;
    for (std::size_t n = 1; n < targetEnd; ++n) {
        loopDist += graph[edge(targetPath[n - 1], targetPath[n], graph).first].length;
    }

    for (std::size_t n = 1; n < sourceEnd; ++n) {
        loopDist += graph[edge(sourcePath[n - 1], sourcePath[n], graph).first].length;
    }

#ifdef DEBUG_LOOPS
    std::cout << "DEBUG:label_loops_and_trees: Found a loop with distance " << loopDist << " : \n";
#endif

    for (std::size_t n = 0; n < targetEnd; ++n) {
        graph[targetPath[n]].node->setLoop(true);
        graph[targetPath[n]].node->setLoopDistance(loopDist);

#ifdef DEBUG_LOOPS
        std::cout << graph[targetPath[n]].node->getPosition() << '\n';
#endif   // DEBUG_LOOPS
    }

    for (std::size_t n = 0; n < sourceEnd; ++n) {
        graph[sourcePath[n]].node->setLoop(true);
        graph[sourcePath[n]].node->setLoopDistance(loopDist);

#ifdef DEBUG_LOOPS
        std::cout << graph[sourcePath[n]].node->getPosition() << '\n';
#endif   // DEBUG_LOOPS
    }
}

}   // namespace hssh
}   // namespace vulcan
