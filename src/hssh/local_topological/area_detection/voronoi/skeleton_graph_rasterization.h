/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_graph_rasterization.h
* \author   Collin Johnson
*
* Declaration of rasterize_graph_onto_grid for redrawing a skeleton graph onto the place grid.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_RASTERIZATION_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_RASTERIZATION_H

#include "hssh/local_topological/area_detection/voronoi/skeleton_graph.h"

namespace vulcan
{
    
namespace hssh
{
    
class VoronoiSkeletonGrid;

/**
* rasterize_graph_onto_grid takes a pruned skeleton graph and labels all
* the cells that correspond to the vertices and edges as SKELETON_CELL_SKELETON.
*/
void rasterize_graph_onto_grid(const SkeletonGraph& graph, VoronoiSkeletonGrid& grid);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_RASTERIZATION_H
