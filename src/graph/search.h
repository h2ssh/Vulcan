/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     search.h
* \author   Collin Johnson
*
* Declaration of a variety of search algorithms for a Graph. The implementation of the
* algorithms will be in the various *_impl.h files to avoid cluttering this file.
*/

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

namespace vulcan
{
namespace graph
{

// NOTE: Nothing is implemented here right now because the code was looking heinously ugly. The search
//       is currently implemented in planner/goal/search.h using the hssh::TopologicalGraph
//       to get the implementation working before generalizing to a template approach -- if needed later.

}
}

#include "graph/astar_impl.h"

#endif // GRAPH_SEARCH_H
