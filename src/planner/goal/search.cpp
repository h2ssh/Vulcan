/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     search.cpp
 * \author   Collin Johnson
 *
 * Definition of AStarSearch.
 */

#include "planner/goal/search.h"
#include <algorithm>
#include <cassert>
#include <iostream>

// #define DEBUG_ASTAR

namespace vulcan
{
namespace planner
{

bool compareNodes(AStarSearch::astar_node_t* const& lhs, AStarSearch::astar_node_t* const& rhs)
{
    return (lhs->costSoFar + lhs->costToGo) < (rhs->costSoFar + rhs->costToGo);
}


AStarSearch::AStarSearch(const hssh::TopologicalGraph& graph) : searchQueue(compareNodes), topoGraph(graph)
{
    createNodesForGraph();
}


bool AStarSearch::search(const hssh::TopologicalVertex& start, const hssh::TopologicalVertex& goal)
{
    // The search is only valid if both the start and goal nodes exist in the graph, otherwise there
    // is no point in even attempting a search
    if ((vertexToNode.find(start.getId()) != vertexToNode.end())
        && (vertexToNode.find(goal.getId()) != vertexToNode.end())) {
        initializeSearch(start, goal);
        return performSearch();
    } else {
        return false;
    }
}


void AStarSearch::createNodesForGraph(void)
{
    const std::vector<Vertex>& vertices = topoGraph.getVertices();

    for (size_t n = 0; n < vertices.size(); ++n) {
        astar_node_t* vertexNode = nodes.newObject();

        vertexNode->vertex = &vertices[n];
        vertexToNode.insert(std::make_pair(vertices[n].getId(), vertexNode));
    }
}


void AStarSearch::initializeSearch(const Vertex& start, const Vertex& goal)
{
    searchQueue.clear();
    expanded.clear();
    this->goal = goal;

    for (auto nodeIt = vertexToNode.begin(), nodeEnd = vertexToNode.end(); nodeIt != nodeEnd; ++nodeIt) {
        nodeIt->second->parent = 0;
        nodeIt->second->costSoFar = 0.0;
        nodeIt->second->costToGo = distance_between_points(nodeIt->second->vertex->getPosition(), goal.getPosition());
    }

    assert(vertexToNode.find(start.getId()) != vertexToNode.end());
    assert(vertexToNode.find(goal.getId()) != vertexToNode.end());

    searchQueue.insert(vertexToNode[start.getId()]);

    debug.expansionSequence.clear();
}


bool AStarSearch::performSearch(void)
{
    assert(searchQueue.size());

    astar_node_t* currentNode = 0;

    while (searchQueue.size()) {
        currentNode = searchQueue.extract();

        debug.expansionSequence.push_back(*(currentNode->vertex));

        // If we have hit the goal, then all done. Nothing more needs to be done.
        if (isGoalNode(currentNode)) {
            break;
        }

        if (expanded.find(currentNode->vertex->getId()) == expanded.end()) {
            expanded.insert(currentNode->vertex->getId());
            expandNode(currentNode);
        }
#ifdef DEBUG_ASTAR
        else {
            std::cout << "DEBUG:AStarSearch: Popped already expanded node off the queue: "
                      << currentNode->vertex->getId() << '\n';
        }
#endif
    }

    if (isGoalNode(currentNode)) {
        extractPathFromGoalNode(currentNode);
        debug.path = path;
        return true;
    } else {
        return false;
    }
}


void AStarSearch::expandNode(astar_node_t* node)
{
#ifdef DEBUG_ASTAR
    std::cout << "DEBUG:AStarSearch:Expanding node " << node->vertex->getId() << '\n';
#endif

    const std::vector<hssh::TopologicalEdge>& edges = topoGraph.getVertexEdges(*(node->vertex));

    for (auto edgeIt = edges.begin(), edgeEnd = edges.end(); edgeIt != edgeEnd; ++edgeIt) {
        queueNeighborIfLessCost(node, *edgeIt);
    }
}


void AStarSearch::queueNeighborIfLessCost(astar_node_t* parent, const Edge& edge)
{
    double costToNode = parent->costSoFar + parent->vertex->getCost() + edge.getCost();
    const Vertex& neighbor = (edge.getStart().getId() == parent->vertex->getId()) ? edge.getEnd() : edge.getStart();

    if (expanded.find(neighbor.getId()) != expanded.end()) {
        return;
    }

    assert(vertexToNode.find(neighbor.getId()) != vertexToNode.end());

    astar_node_t* node = vertexToNode[neighbor.getId()];

    // If there is no parent, then this is first path to reach this node. If costToNode is less than the cost of a
    // previous path to reach the node, then update the costToNode and push this node into the queue Because the queue
    // maintains pointers, the two nodes become identical, but that's fine in the queue because the expanded list will
    // keep both instances from being expanded
    if (!node->parent) {
        node->costSoFar = costToNode;
        node->parent = parent;

        searchQueue.insert(node);
    } else if (costToNode < node->costSoFar) {
#ifdef DEBUG_ASTAR
        std::cout << "DEBUG:AStarSearch:Changing parent for " << neighbor.getId() << " from "
                  << node->parent->vertex->getId() << " to " << parent->vertex->getId()
                  << " Old cost:" << node->costSoFar << " New cost:" << costToNode << '\n';
#endif

        node->costSoFar = costToNode;
        node->parent = parent;

        searchQueue.fix();
    }
}


void AStarSearch::extractPathFromGoalNode(astar_node_t* goalNode)
{
    std::vector<Vertex> pathVertices;

    while (goalNode) {
        pathVertices.push_back(*(goalNode->vertex));

        goalNode = goalNode->parent;
    }

    std::reverse(pathVertices.begin(), pathVertices.end());

    path = graph::Path<Vertex>(pathVertices);
}

}   // namespace planner
}   // namespace vulcan
