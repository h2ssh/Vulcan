/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug_info.h
* \author   Collin Johnson
*
* Declaration of graph_search_debug_info_t and goal_debug_info_t.
*/

#ifndef PLANNER_GOAL_DEBUG_INFO_H
#define PLANNER_GOAL_DEBUG_INFO_H

#include <hssh/global_topological/graph.h>
#include <graph/path.h>
#include <vector>

namespace vulcan
{
namespace planner
{

struct graph_search_debug_info_t
{
    std::vector<hssh::TopologicalVertex> expansionSequence;
    graph::Path<hssh::TopologicalVertex> path;
    hssh::TopologicalGraph               graph;
};

struct goal_debug_info_t
{
    graph_search_debug_info_t searchInfo;
};

}
}

#endif // PLANNER_GOAL_DEBUG_INFO_H
