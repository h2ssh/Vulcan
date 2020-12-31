/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     all_paths.cpp
 * \author   Collin Johnson
 *
 * Definition of AllPaths graph reducer.
 */

#include "math/graph/all_paths.h"

namespace vulcan
{
namespace math
{

AllPaths::AllPaths(bool useCachedGraph) : useVertexPool(useCachedGraph)
{
}


PathGraph AllPaths::extractAllPathsSubgraph(const PathGraph& graph, const std::vector<path_vertex_t*>& vertices)
{
    PathGraph subgraph;

    initializeSearch(vertices);
    runSearch(graph, subgraph);

    return subgraph;
}


void AllPaths::initializeSearch(const std::vector<path_vertex_t*>& vertices)
{
    pathNodePool.reset();
    vertexPool.reset();
    activeNodes.clear();
    expandedNodes.clear();

    all_paths_node_t* initialNode = 0;

    for (int n = vertices.size(); --n >= 0;) {
        initialNode = newNode(vertices[n], vertices[n], 0);
        initialNode->numParents = 0;

        activeNodes.insert(std::make_pair(vertices[n], initialNode));

        nodeQueue.push(initialNode);
    }
}


void AllPaths::runSearch(const PathGraph& graph, PathGraph& subgraph)
{
    while (!nodeQueue.empty()) {
        all_paths_node_t* currentNode = nodeQueue.front();

        expandNode(currentNode, subgraph);

        nodeQueue.pop();
    }
}


void AllPaths::expandNode(all_paths_node_t* node, PathGraph& subgraph)
{
    for (int n = node->vertex->numAdjacent; --n >= 0;) {
        path_vertex_t* currentVertex = node->vertex->adjacent[n];

        // If not looking at the parent of this node, then the node needs to be expanded. Either a new potential
        // connection in the subgraph is made. Or the current path is copied to the subgraph if the node is a collision
        // between two paths
        if (!isParentNode(node, currentVertex) && expandedNodes.find(currentVertex) == expandedNodes.end()) {
            std::map<path_vertex_t*, all_paths_node_t*>::iterator nodeIt = activeNodes.find(currentVertex);

            // If the node already exists, one of two possibilities exists:
            // The node has the same source, in which case, this node should be made a parent of the node, which is
            // safe because it is guaranteed the node isn't a parent of the currently expanding node
            // The node has a different source. Add a vertex to the subgraph that goes between the two nodes and then
            // recursively add all nodes along the path to the subgraph
            if (nodeIt != activeNodes.end()) {
                handleActiveNodeEdge(node, nodeIt->second, subgraph);
            } else {
                handleNewChildVertex(node, currentVertex);
            }
        }
    }

    expandedNodes.insert(node->vertex);
}


bool AllPaths::isParentNode(all_paths_node_t* node, path_vertex_t* vertex)
{
    for (int n = node->numParents; --n >= 0;) {
        if (node->parents[n]->vertex == vertex) {
            return true;
        }
    }

    return false;
}


void AllPaths::handleActiveNodeEdge(all_paths_node_t* parent, all_paths_node_t* child, PathGraph& subgraph)
{
    // When two active nodes collide, then a path has been found through the graph, so trace back the parent
    // pointers for each node and add them all to the subgraph.
    if (parent->source != child->source) {
        addEdgeBetweenNodes(parent, child, subgraph);

        addPathToSubgraph(parent, subgraph);
        addPathToSubgraph(child, subgraph);
    }
}


void AllPaths::handleNewChildVertex(all_paths_node_t* parent, path_vertex_t* childVertex)
{
    all_paths_node_t* childNode = newNode(parent->source, childVertex, parent);

    activeNodes.insert(std::make_pair(childVertex, childNode));

    nodeQueue.push(childNode);
}


void AllPaths::addEdgeBetweenNodes(all_paths_node_t* leftNode, all_paths_node_t* rightNode, PathGraph& subgraph)
{
    // NOTE: left and right don't really have any meaning here beyond being ways to easily distinguish between
    //       the nodes
    if (leftNode->subgraphVertex == 0) {
        leftNode->subgraphVertex = newSubgraphVertex(leftNode->vertex);

        subgraph.addVertex(leftNode->subgraphVertex);
    }

    if (rightNode->subgraphVertex == 0) {
        rightNode->subgraphVertex = newSubgraphVertex(rightNode->vertex);

        subgraph.addVertex(rightNode->subgraphVertex);
    }

    path_vertex_t* leftVertex = leftNode->subgraphVertex;
    path_vertex_t* rightVertex = rightNode->subgraphVertex;

    leftVertex->adjacent[leftVertex->numAdjacent++] = rightVertex;
    rightVertex->adjacent[rightVertex->numAdjacent++] = leftVertex;
}


void AllPaths::addPathToSubgraph(all_paths_node_t* node, PathGraph& subgraph)
{
    if (node->pathsAdded) {
        return;
    }

    node->pathsAdded = true;

    for (int n = node->numParents; --n >= 0;) {
        addEdgeBetweenNodes(node, node->parents[n], subgraph);

        // Recursively expand all the parents as you go, order doesn't matter
        addPathToSubgraph(node->parents[n], subgraph);
    }
}


AllPaths::all_paths_node_t* AllPaths::newNode(path_vertex_t* source, path_vertex_t* vertex, all_paths_node_t* parent)
{
    all_paths_node_t* node = pathNodePool.newObject();

    node->parents[0] = parent;
    node->numParents = 1;

    node->source = source;
    node->vertex = vertex;
    node->subgraphVertex = 0;
    node->pathsAdded = false;

    return node;
}


path_vertex_t* AllPaths::newSubgraphVertex(path_vertex_t* graphVertex)
{
    path_vertex_t* vertex = useVertexPool ? vertexPool.newObject() : new path_vertex_t();

    vertex->numAdjacent = 0;
    vertex->point = graphVertex->point;

    return vertex;
}

}   // namespace math
}   // namespace vulcan
