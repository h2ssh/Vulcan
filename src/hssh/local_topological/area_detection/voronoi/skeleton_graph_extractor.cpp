/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_graph_extractor.cpp
* \author   Collin Johnson
*
* Definition of SkeletonGraphExtractor.
*/

#include <hssh/local_topological/area_detection/voronoi/skeleton_graph_extractor.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <cassert>
#include <iostream>

// #define DEBUG_GRAPH_SIZE
// #define DEBUG_VERTICES

namespace vulcan
{
namespace hssh
{

cell_t find_vertex_along_edge(cell_t                      start,
                              const Point<int16_t>& direction,
                              VoronoiSkeletonGrid&        grid);
bool is_node_vertex(cell_t                      node,
                    const Point<int16_t>& direction,
                    const VoronoiSkeletonGrid&  grid);


SkeletonGraphExtractor::SkeletonGraphExtractor(float minVertexDistance)
: minVertexDistance_(minVertexDistance)
{
}


SkeletonGraphExtractor::~SkeletonGraphExtractor(void)
{
}


SkeletonGraph SkeletonGraphExtractor::extractGraph(const std::set<cell_t>& startCells, VoronoiSkeletonGrid& grid)
{
    initializeExtraction();

    for(auto cell : startCells)
    {
        skeleton_graph_vertex_t* initialVertex = newSkeletonVertex(cell);
        activeVertices.insert(std::make_pair(cell, initialVertex));
        vertexQueue.push(initialVertex);
    }

    assert(vertexQueue.size() > 0);

    runExtraction(grid);

    // Convert the active vertices into a SkeletonGraph
    SkeletonGraph graph;
    for(auto graphIt = activeVertices.begin(), endIt = activeVertices.end(); graphIt != endIt; ++graphIt)
    {
        graph.addVertex(graphIt->second);
    }

#ifdef DEBUG_GRAPH_SIZE
    std::cout<<"INFO: Extractor: Vertices:"<<numVertices<<" Edges:"<<numEdges<<'\n';
#endif

    return graph;
}


void SkeletonGraphExtractor::initializeExtraction(void)
{
    activeVertices.clear();
    vertexPool.reset();

    numEdges    = 0;
    numVertices = 0;
}


void SkeletonGraphExtractor::runExtraction(VoronoiSkeletonGrid& grid)
{
    skeleton_graph_vertex_t* vertex = 0;

    while(!vertexQueue.empty())
    {
        vertex = vertexQueue.front();

#ifdef DEBUG_VERTICES
        std::cout<<"INFO: Extractor: Vertex:"<<vertex->point<<'\n';
#endif

        expandVertex(vertex, grid);

        vertexQueue.pop();
    }
}


void SkeletonGraphExtractor::expandVertex(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid)
{
    if(grid.getMetricDistance(vertex->point.x, vertex->point.y) >= minVertexDistance_)
    {
        // Expand to all children
        findAdjacentVertices(vertex, grid);

        // Mark the TEMP big, so it doesn't get queued again by
        // children that get added
        grid.addClassification(vertex->point.x, vertex->point.y, SKELETON_CELL_TEMP);
    }
}


void SkeletonGraphExtractor::findAdjacentVertices(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid)
{
    NeighborArray neighbors;
    std::size_t numNeighbors = neighbor_cells_with_classification(vertex->point, SKELETON_CELL_SKELETON, grid, FOUR_THEN_EIGHT_WAY, neighbors);
    
    for(std::size_t n = 0; n < numNeighbors; ++n)
    {
        considerVertex(vertex, neighbors[n], neighbors[n] - vertex->point, grid);
    }
}


bool SkeletonGraphExtractor::considerVertex(skeleton_graph_vertex_t*    parent,
                                            cell_t                      pointToConsider,
                                            const Point<int16_t>& vertexDirection,
                                            VoronoiSkeletonGrid&        grid)
{
    // Only need to consider SKELETON. Everything else is unimportant.
    uint8_t vertexClassification = grid.getClassification(pointToConsider.x, pointToConsider.y);
    bool considered = vertexClassification & SKELETON_CELL_SKELETON;

    // Ignore those marked temp, as that means the cell has been visited already
    if(considered && !(vertexClassification & SKELETON_CELL_TEMP))
    {
        // Once a new vertex has been found, need to check if it exists already. If so, then just use
        // the existing vertex, otherwise create a new one and add it to the active list
//         pointToConsider = find_vertex_along_edge(pointToConsider, vertexDirection, grid);

        auto mapVertex = activeVertices.find(pointToConsider);

        skeleton_graph_vertex_t* newVertex = 0;

        if(mapVertex != activeVertices.end())
        {
            newVertex = mapVertex->second;

#ifdef DEBUG_VERTICES
            std::cout<<"INFO: Extractor: Considered was active:"<<pointToConsider<<'\n';
#endif
        }
        else
        {
            newVertex = newSkeletonVertex(pointToConsider);
            activeVertices.insert(std::make_pair(pointToConsider, newVertex));

            // Only push the new vertex in the case that a new vertex is created. If already in the map,
            // then already in the queue, so leave it be.
            vertexQueue.push(newVertex);

#ifdef DEBUG_VERTICES
            std::cout<<"INFO: Extractor: Considered was new:"<<pointToConsider<<'\n';
#endif

            ++numVertices;
        }

        if(addEdge(parent, newVertex))
        {
            ++numEdges;
        }
    }

    return considered;
}


bool SkeletonGraphExtractor::addEdge(skeleton_graph_vertex_t* parent, skeleton_graph_vertex_t* child)
{
    // Adding a vertex simply links the child and the parent together with a new edge
    float edgeDistance = distance_between_points(parent->point, child->point);

    assert(parent->numAdjacent < MAX_ADJACENT_VERTICES);
    assert(child->numAdjacent < MAX_ADJACENT_VERTICES);

    parent->distance[parent->numAdjacent]   = edgeDistance;
    parent->adjacent[parent->numAdjacent++] = child;

    child->distance[child->numAdjacent]   = edgeDistance;
    child->adjacent[child->numAdjacent++] = parent;
    
    return true;
}


skeleton_graph_vertex_t* SkeletonGraphExtractor::newSkeletonVertex(const Point<uint16_t>& point)
{
    skeleton_graph_vertex_t* newVertex = vertexPool.newObject();

    newVertex->point       = point;
    newVertex->numAdjacent = 0;

    return newVertex;
}


cell_t find_vertex_along_edge(cell_t                      node,
                              const Point<int16_t>& direction,
                              VoronoiSkeletonGrid&        grid)
{
    // Use a recursive formulation for finding the vertex.
    if(is_node_vertex(node, direction, grid))
    {
        return node;
    }

    // If along an edge, then recurse to find the edge vertex. Add TEMP bit before passing on the vertex value
    // so the edge doesn't get explored again
    // Recursion baby!
    cell_t edgeVertex = find_vertex_along_edge(node + direction,
                                               direction,
                                               grid);

    grid.addClassification(node.x, node.y, SKELETON_CELL_TEMP);
    
    return edgeVertex;
}


bool is_node_vertex(cell_t                      node,
                    const Point<int16_t>& direction,
                    const VoronoiSkeletonGrid&  grid)
{
    // The invariants are:
    // The cell in -direction is SKELETON, as that is the parent of this cell, it must hold
    // A cell is a vertex if node+direction != SKELETON or the count of SKELETON cells around the node
    // is greater than 2 (meaning a cell not on the edge is SKELETON indicating two incoming edges to
    // the node

    // Bottomed out by reaching the edge of the grid
    int width  = grid.getWidthInCells();
    int height = grid.getHeightInCells();
    if(node.x == 0 || node.x+1 == width || node.y == 0 || node.y+1 == height)
    {
        return true;
    }
    
    if(!(grid.getClassification(node.x+direction.x, node.y+direction.y) & SKELETON_CELL_SKELETON))
    {
        return true;
    }
    
    NeighborArray neighbors;
    return neighbor_cells_with_classification(node, SKELETON_CELL_SKELETON, grid, FOUR_WAY, neighbors) > 2;
}

} // namespace hssh
} // namespace vulcan
