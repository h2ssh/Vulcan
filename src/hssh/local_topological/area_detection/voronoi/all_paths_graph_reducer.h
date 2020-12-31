/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     all_paths_graph_reducer.h
* \author   Collin Johnson
*
* Declaration of AllPathsGraphReducer.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ALL_PATHS_GRAPH_REDUCER_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ALL_PATHS_GRAPH_REDUCER_H

#include "math/graph/all_paths.h"
#include "hssh/local_topological/area_detection/voronoi/skeleton_graph_reducer.h"
#include <string>

namespace vulcan
{
namespace hssh
{

const std::string ALL_PATHS_GRAPH_REDUCER_TYPE("all_paths");

/**
* AllPathsGraphReducer reduces the skeleton graph by creating a subgraph of the extended
* Voronoi graph that contains only the edges that belong to a path connecting the exit points.
* The all paths reducer includes ALL paths between exit points, which may be an optimistic
* view of things, if that is the case, a different reducer can be implemented.
*/
class AllPathsGraphReducer : public SkeletonGraphReducer
{
public:

    /**
    * Constructor for AllPathsGraphReducer.
    */
    AllPathsGraphReducer(void);

    /**
    * reduceSkeleton takes a SkeletonGraph in the form of an extended Voronoi graph and
    * creates a reduced extended Voronoi graph representation. The output graph maintains
    * connectivity between the provided exit points while pruning out all edges that
    * terminate at non-exit points.
    *
    * The memory contained in the returned SkeletonGraph is owned by the SkeletonGraphReducer
    * instance and should not be freed by the callee.
    *
    * \param    graph           Extended Voronoi graph to be reduced
    * \param    exitPoints      Vertices in the provided graph that are the exit points of interest
    * \return   A reduced extended Voronoi graph.
    */
    virtual const SkeletonGraph& reduceSkeleton(const SkeletonGraph& graph, const std::vector<skeleton_graph_vertex_t*>& exitPoints);

private:

    math::AllPaths graphReducer;
    SkeletonGraph  reduced;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ALL_PATHS_GRAPH_REDUCER_H
