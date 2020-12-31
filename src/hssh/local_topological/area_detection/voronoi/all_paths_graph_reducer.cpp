/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     all_paths_graph_reducer.cpp
* \author   Collin Johnson
*
* Definition of AllPathsGraphReducer.
*/

#include "hssh/local_topological/area_detection/voronoi/all_paths_graph_reducer.h"

namespace vulcan
{
namespace hssh
{

AllPathsGraphReducer::AllPathsGraphReducer(void) : graphReducer(true)
{
}


const SkeletonGraph& AllPathsGraphReducer::reduceSkeleton(const SkeletonGraph& graph, const std::vector<skeleton_graph_vertex_t*>& exitPoints)
{
    reduced = graphReducer.extractAllPathsSubgraph(graph, exitPoints);
    
    return reduced;
}

}
}
