/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     graph.h
* \author   Collin Johnson
*
* Declaration of a function to convert a topological map into a graph so
* graph-based algorithms can be run on a map.
*
* Declaration of types and typedefs used to represent a topological map as a graph.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_GRAPH_H
#define HSSH_GLOBAL_TOPOLOGICAL_GRAPH_H

#include "graph/edge.h"
#include "graph/vertex.h"
#include "graph/edge_ordered_graph.h"

namespace vulcan
{
namespace hssh
{

class  GlobalPlace;
class  TopologicalMap;
struct GlobalLocation;

struct NodeData
{
    int  id;
    bool isSegment;

    explicit NodeData(int id = -1, bool isSegment = false)
    : id(id)
    , isSegment(isSegment)
    {
    }
};

using TopologicalVertex = graph::Vertex<NodeData>;
using TopologicalEdge = graph::Edge<TopologicalVertex>;
using TopologicalGraph = graph::EdgeOrderedGraph<TopologicalVertex, TopologicalEdge>;

/**
* convert_map_to_graph converts a topological map into an edge-ordered graph to allow
* for further processing.
*
* \param    map         Map to be converted
* \return   EdgeOrderedGraph where vertices are GlobalPlaces and edges are GlobalPathSegments.
*/
TopologicalGraph convert_map_to_graph(const TopologicalMap& map);

/**
* convert_location_to_vertex converts the global location in a topological map into a vertex
* in the graph representation of that map.
*
* \param    location    Location to be converted
* \param    map         Map in which the location resides
* \return   TopologicalVertex representing the location.
*/
TopologicalVertex convert_location_to_vertex(const GlobalLocation& location, const TopologicalMap& map);

/**
* convert_place_to_vertex converts a GlobalPlace in the topological map into a TopologicalVertex.
*
* \param    place       Place to convert
* \param    map         Map in which the place exists
* \return   TopologicalVertex representing the place.
*/
TopologicalVertex convert_place_to_vertex(const GlobalPlace& place, const TopologicalMap& map);


} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_GRAPH_H
