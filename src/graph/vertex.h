/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     vertex.h
 * \author   Collin Johnson
 *
 * Definition of Vertex, a default implementation of a class satisfying the
 * Vertex policy used by the Graph class.
 */

#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include "core/point.h"

namespace vulcan
{
namespace graph
{

/**
 * Vertex is a default implementation of a class that satisfies the policy for a class
 * that can be used as a Vertex for the Graph class. The Vertex policy requires the following
 * to be implemented:
 *
 *   - operator==           : need to be able to check equality of nodes
 *   - operator<            : need to be able to sort vertices/put them in an associative container
 *
 * Additionally, the Vertex class has an id, which is used for determining equality, and a cost to use for
 * planning within a graph. The Vertex class is a template parameterized with VertexData, which allows
 * a user to attach data to a Vertex
 * for later retrieval.
 */
template <typename VertexData>
class Vertex
{
public:
    /**
     * Default constructor for Vertex.
     */
    Vertex(void) { }

    /**
     * Constructor for Vertex.
     *
     * \param    id          Id of the vertex
     * \param    position    Position of the vertex in an embedding of the graph
     * \param    data        Data to attach to the vertex
     * \param    cost        Cost of the vertex (default = 0)
     */
    Vertex(int id, const Point<float>& position, VertexData data, double cost = 0.0)
    : id(id)
    , position(position)
    , data(data)
    , cost(cost)
    {
    }

    /**
     * getId retrieves the id of the vertex.
     */
    int getId(void) const { return id; }

    /**
     * getPosition retrieves the (x,y) position of the vertex in an embedding of the graph.
     */
    Point<float> getPosition(void) const { return position; }

    /**
     * getVertexData retrieves the data associated with the vertex.
     */
    VertexData getVertexData(void) const { return data; }

    /**
     * getCost retrieves the cost of visiting this node.
     */
    double getCost(void) const { return cost; }

    // Operator overloads
    bool operator==(const Vertex& rhs) const { return id == rhs.id; }

    bool operator<(const Vertex& rhs) const { return id < rhs.id; }

private:
    int id;
    Point<float> position;
    VertexData data;
    double cost;
};

}   // namespace graph
}   // namespace vulcan

#endif   // GRAPH_VERTEX_H
