/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     edge_ordered_graph.h
* \author   Collin Johnson
*
* Definition of EdgeOrderedGraph, a subclass of Graph that allows for easy creation of
* edge-ordered graphs for fast checking of map planarity.
*/

#ifndef GRAPH_EDGE_ORDERED_GRAPH_H
#define GRAPH_EDGE_ORDERED_GRAPH_H

#include <graph/graph.h>

namespace vulcan
{
namespace graph
{

/**
* EdgeOrderedGraph
*/
template <class Vertex, class Edge>
class EdgeOrderedGraph : public Graph<Vertex, Edge>
{
public:

    // Graph interface
    virtual void addVertex(const Vertex& vertex)
    {
        Graph<Vertex, Edge>::addVertex(vertex);
    }

    virtual void addEdge(const Edge& edge)
    {
        Graph<Vertex, Edge>::addEdge(edge);
    }
};

}
}

#endif // GRAPH_EDGE_ORDERED_GRAPH_H
