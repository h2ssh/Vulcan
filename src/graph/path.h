/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path.h
* \author   Collin Johnson
*
* Definition of Path.
*/

#ifndef GRAPH_PATH_H
#define GRAPH_PATH_H

#include <vector>

namespace vulcan
{
namespace graph
{

/**
* Path represents a path through a graph. The Path is an ordered sequence of vertices.
*/
template <typename Vertex>
class Path
{
public:

    /**
    * Default constructor for Path.
    */
    Path(void)
        : cost(0.0)
    {
    }

    /**
    * Constructor for Path.
    *
    * If the cost is not specified, it will be calculated by traversing the nodes in the path.
    *
    * \param    path            Sequence of nodes that are the path
    * \param    cost            Cost of the path (optional)
    */
    Path(const std::vector<Vertex>& path, double cost = 0.0)
        : path(path)
        , cost(cost)
    {
        if(cost == 0.0)
        {
            for(auto vertexIt = path.begin(), vertexEnd = path.end(); vertexIt != vertexEnd; ++vertexIt)
            {
                this->cost += vertexIt->getCost();
            }
        }
    }

    /**
    * getLength retrieves the number of vertices in the path.
    */
    size_t getLength(void) const { return path.size(); }

    /**
    * getPath retrieves the sequence of vertices that make up the path.
    */
    const std::vector<Vertex>& getPath(void) const { return path; }

    /**
    * getCost retrieves the cost of traversing the path through the graph.
    */
    double getCost(void) const { return cost; }

private:

    std::vector<Vertex> path;
    double              cost;
};

}
}

#endif // GRAPH_PATH_H
