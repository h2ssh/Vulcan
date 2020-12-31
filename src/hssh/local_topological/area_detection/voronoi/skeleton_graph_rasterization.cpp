/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     skeleton_graph_rasterization.cpp
 * \author   Collin Johnson
 *
 * Definition of rasterize_graph_onto_grid function.
 */

#include "hssh/local_topological/area_detection/voronoi/skeleton_graph_rasterization.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include <cstdlib>

namespace vulcan
{
namespace hssh
{

// Recursive function to draw all the edges and vertices in a depth-first fashion
void rasterize_vertex_edges(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid);
// Fill in all the cells along the edge from start->end (including end). Return
// true if the edge has already been filled.
bool rasterize_edge(skeleton_graph_vertex_t* start, skeleton_graph_vertex_t* end, VoronoiSkeletonGrid& grid);


void rasterize_graph_onto_grid(const SkeletonGraph& graph, VoronoiSkeletonGrid& grid)
{
    const std::vector<skeleton_graph_vertex_t*>& vertices = graph.getVertices();

    for (int n = vertices.size(); --n >= 0;) {
        rasterize_vertex_edges(vertices[n], grid);
    }
}


void rasterize_vertex_edges(skeleton_graph_vertex_t* vertex, VoronoiSkeletonGrid& grid)
{
    for (int i = vertex->numAdjacent; --i >= 0;) {
        rasterize_edge(vertex, vertex->adjacent[i], grid);
    }
}


// Returns whether a SKELETON cell was encountered while rasterizing this edge. If a SKELETON cell was encountered,
// then some part of the edge has been rasterized already, so there is no need for further expansion of vertices
// along the edge
bool rasterize_edge(skeleton_graph_vertex_t* start, skeleton_graph_vertex_t* end, VoronoiSkeletonGrid& grid)
{
    // To rasterize, find the step, then move along the step from start until end is reached
    int xDiff = static_cast<int>(end->point.x) - start->point.x;
    int yDiff = static_cast<int>(end->point.y) - start->point.y;

    // Normalize the step sizes if needed
    if (xDiff != 0) {
        xDiff /= std::abs(xDiff);
    }

    if (yDiff != 0) {
        yDiff /= std::abs(yDiff);
    }

    bool hadSkeletonCell = false;

    uint16_t currentX = start->point.x;
    uint16_t currentY = start->point.y;

    grid.addClassification(currentX, currentY, SKELETON_CELL_REDUCED_SKELETON);

    while ((currentX != end->point.x) || (currentY != end->point.y)) {
        if (currentX != end->point.x) {
            currentX += xDiff;
        }

        if (currentY != end->point.y) {
            currentY += yDiff;
        }

        //         if(grid.getClassification(currentX, currentY) & SKELETON_CELL_REDUCED_SKELETON)
        //         {
        //             hadSkeletonCell = true;
        //             break;
        //         }
        //         else
        //         {
        grid.addClassification(currentX, currentY, SKELETON_CELL_REDUCED_SKELETON);
        //         }
    }

    return !hadSkeletonCell;
}

}   // namespace hssh
}   // namespace vulcan
