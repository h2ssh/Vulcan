/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_GRAPH_GRAPH_H
#define MATH_GRAPH_GRAPH_H

#include <stdint.h>
#include <vector>
#include <core/point.h>

namespace vulcan
{
namespace math
{

/*
* NOTE: I really really don't like how this graph is implemented at the moment, but
*       I needed some data structure that I could start to play with. It will change
*       for sure at some point in the future once I need a graph that contains information
*       other than just points. Once the global topology begins to emerge, it will
*       become especially useful. Right now, this Graph is pretty damn useless and
*       essentially is just a wrapper around std::vector<graph_vertex_t*>. The fact that
*       the bare pointers are floating around is a BAD sign. Necessity will win
*       for now.
*/
    
/**
* graph_vertex_t is a vertex in the graph. A vertex is known by its Point location
* and the connected vertices and the distance to them.
*
* T = the type of point
* N = the maximum number of neighbors
*/
template <typename T, int N>
struct graph_vertex_t
{
    graph_vertex_t(void) :
                numAdjacent(0)
    {    
    }
    
    Point<T> point;
    
    uint8_t numAdjacent;
    
    graph_vertex_t<T, N>* adjacent[N];
    float                 distance[N];
};


/**
* Graph is a simple graph structure that is represented by vertices and edges. At the moment
* Graph is used only to represent the abstract concept of a graph in words. It does not manage
* the memory of the vertices it contains and the edges are currently only implied by the 
* adjacent pointers contained in a graph_vertex_t. I need something right now to advance the
* state of the code, so it'll have to do until I can take the time to do something better.
*/
template <typename T, int N>
class Graph
{
public:
    
    /**
    * addVertex adds a vertex to the graph.
    */
    void addVertex(graph_vertex_t<T, N>* vertex)
    {
        vertices.push_back(vertex);
    }
    
    /**
    * getVertices retrieves all the vertices in the graph. This method is obviously dangerous
    * as messing with the vertices can break the graph. The need for this method is why I need
    * to design a true Graph data structure.
    */
    const std::vector<graph_vertex_t<T, N>*>& getVertices(void) const
    {
        return vertices;
    }
    
    /**
    * size retrieves the size of the graph, which is the number of vertices it contains.
    */
    std::size_t size(void) const { return vertices.size(); }
    
private:
    
    std::vector<graph_vertex_t<T, N>*> vertices;
};

}
}

#endif // MATH_GRAPH_GRAPH_H
