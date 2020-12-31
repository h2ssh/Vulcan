/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topo_graph.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalTopoGraph.
 */

#include "hssh/local_topological/local_topo_graph.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/area_detection/voronoi/search.h"
#include "hssh/local_topological/local_topo_map.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

// #define DEBUG_PATH

namespace vulcan
{
namespace hssh
{

const int kSearchNodeId = -1;


struct SearchNodeData
{
    LTGraphType::vertex_descriptor vertex;
    std::vector<LTGraphType::edge_descriptor> edges;
};

LTGraphType build_skeleton_graph(const VoronoiSkeletonGrid& skeleton);

SearchNodeData add_search_node(LocalTopoGraph::SearchNode node,
                               const std::vector<LTGraphType::vertex_descriptor>& areaNodes,
                               const VoronoiSkeletonGrid& skeleton,
                               LTGraphType& graph);
void remove_search_node(const SearchNodeData& nodeData, LTGraphType& graph);


LocalTopoGraph::LocalTopoGraph(const LocalTopoMap& map) : map_(map)
{
    std::unordered_map<int, LTGraphType::vertex_descriptor> gatewayToVertex;

    // For each area, create vertices for the gateways and add edges between each pair of gateways
    for (auto& area : map_) {
        std::vector<Gateway> areaGateways = area->gateways();

        // Add all gateways
        for (const auto& g : areaGateways) {
            // Validate that we haven't loaded some degenerate gateway
            auto path = path_to_skeleton(g.skeletonCell(), SKELETON_CELL_REDUCED_SKELETON, map_.voronoiSkeleton());

            // Ignore any gateways for which a proper path to the skeleton can't be found.
            if (path.result != VoronoiPathResult::success) {
                continue;
            }

            if (gatewayToVertex.find(g.id()) == gatewayToVertex.end()) {
                LTGVertex vertex;
                vertex.position = g.center();
                vertex.gatewayId = g.id();
                gatewayToVertex[g.id()] = boost::add_vertex(vertex, graph_);
                gateways_[g.id()] = g;
            }

            areaVertices_[area->id()].push_back(gatewayToVertex[g.id()]);
        }

        // Add an edge between all pairs of gateways
        for (std::size_t n = 0; n < areaGateways.size(); ++n) {
            for (std::size_t i = n + 1; i < areaGateways.size(); ++i) {
                LTGEdge edge;
                edge.areaId = area->id();

                // Find the path between the gateways
                auto path = find_path_along_skeleton(areaGateways[n].skeletonCell(),
                                                     areaGateways[i].skeletonCell(),
                                                     SKELETON_CELL_REDUCED_SKELETON,
                                                     map_.voronoiSkeleton());

                if (path.result == VoronoiPathResult::success) {
                    edge.distance = path.length;
                    boost::add_edge(gatewayToVertex.at(areaGateways[n].id()),
                                    gatewayToVertex.at(areaGateways[i].id()),
                                    edge,
                                    graph_);
                } else {
                    std::cerr << "ERROR: LocalTopoGraph: Failed to find path between gateways: " << areaGateways[n]
                              << " to " << areaGateways[i] << " in " << area->id() << '\n';
                }
            }
        }
    }
}


LocalTopoRoute LocalTopoGraph::findPath(SearchNode start, SearchNode finish)
{
#ifdef DEBUG_PATH
    std::cout << "Searching for path from " << start.first << ':' << start.second << " to " << finish.first << ':'
              << finish.second << '\n';
#endif   // DEBUG_PATH

    // If either of the endpoints isn't in the graph, then there can't be any connection
    if ((areaVertices_.find(start.second) == areaVertices_.end())
        || (areaVertices_.find(finish.second) == areaVertices_.end())) {
        //         std::cout << "ERROR: LocalTopoGraph: Failed to find a start or finish vertex. Cannot plan a
        //         route.\n";
        return LocalTopoRoute();
    }

    // If the two nodes aren't in the same area, then search for a path between areas
    if (start.second != finish.second) {
        return findPathBetweenAreas(start, finish);
    }
    // Otherwise need to just find the path within the area
    else {
        return findPathWithinArea(start, finish);
    }
}


LocalTopoRoute LocalTopoGraph::findPathBetweenAreas(SearchNode start, SearchNode finish)
{
    using namespace boost;

    auto startData = add_search_node(start, areaVertices_.at(start.second), map_.voronoiSkeleton(), graph_);
    auto finishData = add_search_node(finish, areaVertices_.at(finish.second), map_.voronoiSkeleton(), graph_);

    std::vector<double> distances(num_vertices(graph_));
    std::vector<LTGraphType::vertex_descriptor> predecessors(num_vertices(graph_));

    dijkstra_shortest_paths(
      graph_,
      startData.vertex,
      weight_map(get(&LTGEdge::distance, graph_))
        .distance_map(make_iterator_property_map(distances.begin(), get(vertex_index, graph_)))
        .predecessor_map(make_iterator_property_map(predecessors.begin(), get(vertex_index, graph_))));

    auto nextVertex = finishData.vertex;

    std::vector<double> eventDistances;
    std::vector<LocalArea::Id> areaSequence;
    std::vector<Point<float>> eventPointSequence;
    std::vector<int> eventGatewaySequence;

    // Extract the path by following the back pointers and finding the associated edges
    // When the predecessor is the same as the vertex, then the source has been reached
    while (predecessors[nextVertex] != nextVertex) {
        auto parentEdge = edge(predecessors[nextVertex], nextVertex, graph_);
        assert(parentEdge.second);
        areaSequence.push_back(graph_[parentEdge.first].areaId);
        eventDistances.push_back(graph_[parentEdge.first].distance);
        eventPointSequence.push_back(graph_[nextVertex].position);
        eventGatewaySequence.push_back(graph_[nextVertex].gatewayId);

        nextVertex = predecessors[nextVertex];
    }

    // Add the starting point, as it isn't otherwise included in the events. There's one more vertex than edge and
    // area sequence is the edges and event points are the vertexes
    eventPointSequence.push_back(graph_[startData.vertex].position);
    eventGatewaySequence.push_back(kSearchNodeId);

    // Reverse the order of the path so it front start->finish instead of finish->start
    std::reverse(areaSequence.begin(), areaSequence.end());
    std::reverse(eventDistances.begin(), eventDistances.end());
    std::reverse(eventPointSequence.begin(), eventPointSequence.end());
    std::reverse(eventGatewaySequence.begin(), eventGatewaySequence.end());

    LocalTopoRoute path;

    for (std::size_t n = 0; n < areaSequence.size(); ++n) {
        boost::optional<Gateway> entry;
        if (eventGatewaySequence[n] != kSearchNodeId) {
            entry = gateways_[eventGatewaySequence[n]];
        }

        boost::optional<Gateway> exit;
        if (eventGatewaySequence[n + 1] != kSearchNodeId) {
            exit = gateways_[eventGatewaySequence[n + 1]];
        }

        path.addVisit(LocalTopoRouteVisit(map_.areaWithId(areaSequence[n]),
                                          eventPointSequence[n],
                                          eventPointSequence[n + 1],
                                          eventDistances[n],
                                          entry,
                                          exit));
    }

#ifdef DEBUG_PATH
    std::cout << "Distance to finish:" << path.length() << '\n' << "Areas:";
    std::copy(areaSequence.begin(), areaSequence.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << '\n';
#endif   // DEBUG_PATH

    // Once the path is found, erase the added nodes so the next search can be performed on an untainted graph
    remove_search_node(startData, graph_);
    remove_search_node(finishData, graph_);

    return path;
}


LocalTopoRoute LocalTopoGraph::findPathWithinArea(SearchNode start, SearchNode finish)
{
    // If the start and finish are in the same node, then there's only a single visit
    auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(start.first, map_.voronoiSkeleton()),
                                         utils::global_point_to_grid_cell_round(finish.first, map_.voronoiSkeleton()),
                                         SKELETON_CELL_REDUCED_SKELETON,
                                         map_.voronoiSkeleton());

    // Only a single area was visited for this route
    LocalTopoRoute route;
    route.addVisit(LocalTopoRouteVisit(map_.areaWithId(start.second), start.first, finish.first, path.length));
    return route;
}


SearchNodeData add_search_node(LocalTopoGraph::SearchNode node,
                               const std::vector<LTGraphType::vertex_descriptor>& areaNodes,
                               const VoronoiSkeletonGrid& skeleton,
                               LTGraphType& graph)
{
    SearchNodeData nodeData;
    nodeData.vertex = boost::add_vertex(graph);
    graph[nodeData.vertex].gatewayId = kSearchNodeId;
    graph[nodeData.vertex].position = node.first;

    LTGEdge searchEdge;
    searchEdge.areaId = node.second;
    for (const auto& nodeId : areaNodes) {
        auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(node.first, skeleton),
                                             utils::global_point_to_grid_cell_round(graph[nodeId].position, skeleton),
                                             SKELETON_CELL_REDUCED_SKELETON,
                                             skeleton);

        if (path.cells.empty()) {
            std::cerr << "ERROR: LocalTopoGraph: No path exists from cell in area to gateway:" << node.first
                      << " to gateway " << graph[nodeId].position << '\n';
            continue;
        }

        searchEdge.distance = path.length;
        auto addedEdge = boost::add_edge(nodeData.vertex, nodeId, searchEdge, graph);
        if (addedEdge.second) {
            nodeData.edges.push_back(addedEdge.first);
        } else {
            std::cout << "WARNING:LocalTopoGraph: Failed to create edge between " << graph[nodeId].position << " and "
                      << node.first << '\n';
        }
    }

    return nodeData;
}


void remove_search_node(const SearchNodeData& nodeData, LTGraphType& graph)
{
    // Clearing vertex both erases the vertex and all edges in one safe, fell swoop.
    boost::clear_vertex(nodeData.vertex, graph);
}

}   // namespace hssh
}   // namespace vulcan
