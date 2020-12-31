/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     skeleton_graph_reducer.h
 * \author   Collin Johnson
 *
 * Declaration of SkeletonGraphReducer interface and create_skeleton_graph_reducer factory function.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_REDUCER_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_REDUCER_H

#include "hssh/local_topological/area_detection/voronoi/skeleton_graph.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
 * SkeletonGraphReducer is an abstract base class for graph reduction algorithms. A graph reducer takes
 * a SkeletonGraph and produces a reduce form of that graph that contains only edges needed for
 * maintaining connectivity between the provided exit points.
 *
 * A graph reducer must maintain connectivity, but the density of the connectivity does not
 * need to be fixed. By this condition, I mean that solutions containing all edges between
 * exit points, or some subset of all edges are acceptable.
 */
class SkeletonGraphReducer
{
public:
    virtual ~SkeletonGraphReducer(void) { }

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
    virtual const SkeletonGraph& reduceSkeleton(const SkeletonGraph& graph,
                                                const std::vector<skeleton_graph_vertex_t*>& exitPoints) = 0;
};


/**
 * create_skeleton_graph_reducer creates a new SkeletonGraphReducer of the desired type.
 */
std::unique_ptr<SkeletonGraphReducer> create_skeleton_graph_reducer(const std::string& reducerName);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_GRAPH_REDUCER_H
