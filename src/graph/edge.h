/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     edge.h
 * \author   Collin Johnson
 *
 * Definition of Edge used by Graph.
 */

#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

#include <cstdint>

namespace vulcan
{
namespace graph
{

/**
 * Edge defines the class used to represent edges within the Graph class. Each Edge consists
 * of a start and end vertex and a cost for traversing the edge.
 *
 * Right now, the start and end don't mean anything beyond being a way to differentiate the name
 * of the two vertices. If directed graphs are implemented in the future, then the story will change.
 */
template <class Vertex, class EdgeData = int>
class Edge
{
public:
    /**
     * Default constructor for Edge.
     */
    Edge(void) { }

    /**
     * Constructor for Edge.
     *
     * \param    id              Unique id for the edge
     * \param    start           Start vertex for the edge
     * \param    end             End vertex for the edge
     * \param    cost            Traversal cost for the edge
     */
    Edge(uint32_t id, const Vertex& start, const Vertex& end, double cost) : start(start), end(end), cost(cost) { }

    /**
     * Constructor for Edge.
     *
     * \param    id              Unique id for the edge
     * \param    start           Start vertex for the edge
     * \param    end             End vertex for the edge
     * \param    cost            Traversal cost for the edge
     * \param    data            Data to associate with the edge
     */
    Edge(uint32_t id, const Vertex& start, const Vertex& end, double cost, EdgeData data)
    : start(start)
    , end(end)
    , cost(cost)
    , data(data)
    {
    }

    /**
     * getId retrieves the unique id for the edge.
     */
    uint32_t getId(void) const { return id; }

    /**
     * getStart retrieves the start vertex.
     */
    const Vertex& getStart(void) const { return start; }

    /**
     * getEnd retrieves the end vertex.
     */
    const Vertex& getEnd(void) const { return end; }

    /**
     * getCost retrieves the cost of traversing the Edge.
     */
    double getCost(void) const { return cost; }

    /**
     * getEdgeData retrieves the data associated with the edge.
     */
    EdgeData getEdgeData(void) const { return data; }

    // Operator overloads

    /**
     * Two Edges are considered equal if they share the same end vertices.
     *
     *   (start == rhs.start && end == rhs.end) || (start == rhs.end && end == rhs.start)
     *
     * The Edge is undirected, so either matching of vertices means equality.
     */
    bool operator==(const Edge& rhs) const
    {
        return ((start == rhs.start) && (end == rhs.end)) || ((start == rhs.end) && (end == rhs.start));
    }

private:
    uint32_t id;
    Vertex start;
    Vertex end;
    double cost;
    EdgeData data;
};

}   // namespace graph
}   // namespace vulcan

#endif   // GRAPH_EDGE_H
