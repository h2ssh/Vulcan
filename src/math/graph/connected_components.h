/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     connected_components.h
 * \author   Collin Johnson
 *
 * Definition of ConnectedComponents operation for a graph.
 */

#ifndef MATH_GRAPH_CONNECTED_COMPONENTS_H
#define MATH_GRAPH_CONNECTED_COMPONENTS_H

#include "math/graph/graph.h"
#include <set>

namespace vulcan
{
namespace math
{

typedef Graph<uint16_t, 8> PathGraph;
typedef graph_vertex_t<uint16_t, 8> path_vertex_t;


/**
 * ConnectedComponents takes a Graph and splits it into n connected sub-graphs. Each
 * sub-graph is connected, whereas the Graph probably isn't.
 */
class ConnectedComponents
{
public:
    /**
     * connectedComponents finds the connected subgraphs of the input graph. The subgraphs are shallow
     * copies of the vertices in the input graph, so they won't do their own cleanup operations.
     *
     * \param    graph           Graph to be split
     * \return   Connected subgraphs of graph.
     */
    std::vector<PathGraph> connectedComponents(const PathGraph& graph) const;

private:
    void depthFirstSearch(path_vertex_t* node, PathGraph& graph) const;

    mutable std::set<path_vertex_t*> visited;
};

}   // namespace math
}   // namespace vulcan

#endif   // MATH_GRAPH_CONNECTED_COMPONENTS_H
