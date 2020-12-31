/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_graph.h
* \author   Collin Johnson
*
* Typedefs specifying the exact type of graph to be used for the skeleton graph.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_H

#include <cstdint>
#include "math/graph/graph.h"

namespace vulcan
{
namespace hssh
{

const int MAX_ADJACENT_VERTICES = 8;
    
// All SkeletonGraphs have this form, so a typedef makes a lot of sense to hide the
// unnecessary knowledge of the template parameters
typedef math::Graph<uint16_t, MAX_ADJACENT_VERTICES>          SkeletonGraph;
typedef math::graph_vertex_t<uint16_t, MAX_ADJACENT_VERTICES> skeleton_graph_vertex_t;

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_H
