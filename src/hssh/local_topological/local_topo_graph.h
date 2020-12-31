/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_graph.h
* \author   Collin Johnson
* 
* Declaration of LocalTopoGraph and LocalTopoRoute.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_GRAPH_H

#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/local_topo_route.h"
#include "core/point.h"
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace vulcan
{
namespace hssh
{
    
struct LTGVertex
{
    Point<float> position;
    int gatewayId;
};

struct LTGEdge
{
    float distance;
    int areaId;
};

using LTGraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, LTGVertex, LTGEdge>;

/**
* LocalTopoGraph is a graph data structure extracted from the LocalTopoMap. The graph allows for
* searches to be performed amongst 
*/
class LocalTopoGraph
{
public:
    
    using SearchNode = std::pair<Point<float>, int>;

    /**
    * Constructor for LocalTopoGraph.
    * 
    * \param    map         Map from which to extract the graph
    */
    LocalTopoGraph(const LocalTopoMap& map);
    
    /**
    * findPath finds a path through the graph from the provided start position/area to the finish area/position. The 
    * path distance includes the distance to reach the outgoing gateway from the start position and the distance to
    * reach the finish position from the incoming gateway.
    * 
    * If no path is found from the start to finish nodes, then an empty path is returned.
    * 
    * \param    start           Start position/area
    * \param    finish          Finish position/area
    * \return   Path from from start to finish through the graph.
    */
    LocalTopoRoute findPath(SearchNode start, SearchNode finish);
    
private:

    LTGraphType graph_;
    LTGraphType skeletonGraph_;     // a graph of the cells in the skeleton itself
    LocalTopoMap map_;
    
    std::unordered_map<int, Gateway> gateways_;
    std::unordered_map<int, std::vector<LTGraphType::vertex_descriptor>> areaVertices_;
    CellToTypeMap<LTGraphType::vertex_descriptor> cellVertices_;


    LocalTopoRoute findPathBetweenAreas(SearchNode start, SearchNode finish);
    LocalTopoRoute findPathWithinArea(SearchNode start, SearchNode finish);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_GRAPH_H
