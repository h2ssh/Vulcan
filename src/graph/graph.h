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
* Definition of Graph data structure and base class.
*/

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <cstddef>
#include <vector>
#include <map>

namespace vulcan
{
namespace graph
{

/**
* Graph is a standard undirected graph data structure consisting of vertices and edges. Graphs are built
* vertex-by-vertex and edge-by-edge via addVertex() and addEdge() methods. The vertices and edges
* are copied to internal structures, so lightweight classes for these types make the most sense.
*
* Graph is templated by a Vertex class and an Edge class. Vertex classes must implement the following policy
* (there may be additional requirements by algorithms that use the Graph):
*
*   - operator==           : need to be able to check equality of nodes
*   - operator<            : need to be able to sort vertices/put them in an associative container
*
* Vertex defined in graph/vertex.h is a simple implementation of this policy.
*
* Edge classes must implement the following policy (there may be additional requirements by algorithms
* that use the Graph):
*
*   - getStart()            : retrieve the Vertex at one end of the Edge
*   - getEnd()              : retrieve the Vertex at the other end of the Edge
*   - operator==            : check equality between two edges
*
* The type returned by getStart() and getEnd() must be the same type as the Vertex class.
*
* Edge defined in graph/edge.h is a simple implementation of this policy.
*
* The addVertex() and addEdge() methods are virtual to allow the construction of specialized graphs,
* like edge-ordered or directed graphs, via subclassing the Graph class.
*/
template <class Vertex, class Edge>
class Graph
{
public:

    virtual ~Graph(void) { }

    /**
    * addVertex adds a new vertex to the graph. If an edge has already been added with this vertex as an
    * endpoint, then the vertex will have already been added. Otherwise, a new disconnected vertex will
    * be added to the graph.
    */
    virtual void addVertex(const Vertex& vertex);

    /**
    * addEdge adds a new edge to the graph. If either endpoint of the edge is not already in the graph, then
    * a new vertex will be added for the endpoint and associated with this edge. Both endpoints of the edge
    * will be associated with the edge, so a call to getVertexEdges() will include the added edge./
    */
    virtual void addEdge(const Edge& edge);

    /**
    * numVertices retrieves the number of vertices in the graph.
    */
    size_t numVertices(void) const { return vertices.size(); }

    /**
    * numEdges retrieves the number of edges in the graph.
    */
    size_t numEdges(void) const { return edges.size(); }

    /**
    * getVertices retrieves all the vertices in the Graph.
    */
    const std::vector<Vertex>& getVertices(void) const { return vertices; }

    /**
    * getEdges retrieves all the edges in the Graph.
    */
    const std::vector<Edge>& getEdges(void) const { return edges; }

    /**
    * getVertex retrieves the vertex with the specified id.
    */
    Vertex getVertex(int id) const;

    /**
    * getVertexEdges retrieves the edges associated with the provided vertex. In the default Graph implementation,
    * these edges are sorted in the order in which they were added. Other Graph subclasses may place more
    * stringent conditions on the edge ordering.
    */
    const std::vector<Edge>& getVertexEdges(const Vertex& vertex) const;

    /**
    * clear erases all vertices and edges from the Graph.
    */
    void clear(void);

protected:

    std::vector<Vertex>                 vertices;
    std::vector<Edge>                   edges;
    std::map<Vertex, std::vector<Edge>> vertexEdges;
    std::vector<Edge>                   noEdges;  // empty placeholder to allow vertices with no edges to have a valid return value
};


template <class Vertex, class Edge>
void Graph<Vertex, Edge>::addVertex(const Vertex& vertex)
{
    if(vertexEdges.find(vertex) == vertexEdges.end())
    {
        vertices.push_back(vertex);
        vertexEdges.insert(std::make_pair(vertex, std::vector<Edge>()));
    }
}


template <class Vertex, class Edge>
void Graph<Vertex, Edge>::addEdge(const Edge& edge)
{
    addVertex(edge.getStart());
    addVertex(edge.getEnd());

    vertexEdges[edge.getStart()].push_back(edge);
    vertexEdges[edge.getEnd()].push_back(edge);

    edges.push_back(edge);
}


template <class Vertex, class Edge>
Vertex Graph<Vertex, Edge>::getVertex(int id) const
{
    for(auto vertexIt = vertices.begin(), vertexEnd = vertices.end(); vertexIt != vertexEnd; ++vertexIt)
    {
        if(vertexIt->getId() == id)
        {
            return *vertexIt;
        }
    }

    return Vertex();
}


template <class Vertex, class Edge>
const std::vector<Edge>& Graph<Vertex, Edge>::getVertexEdges(const Vertex& vertex) const
{
    auto edgesIt = vertexEdges.find(vertex);

    if(edgesIt != vertexEdges.end())
    {
        return edgesIt->second;
    }

    return noEdges;
}


template <class Vertex, class Edge>
void Graph<Vertex, Edge>::clear(void)
{
    edges.clear();
    vertices.clear();
    vertexEdges.clear();
}

}
}

#endif // GRAPH_GRAPH_H
