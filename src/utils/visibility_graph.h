/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph.h
* \author   Collin Johnson
*
* Declaration of VisibilityGraph.
*/

#ifndef UTILS_VISIBILITY_GRAPH_H
#define UTILS_VISIBILITY_GRAPH_H

#include "core/point_util.h"
#include "utils/visibility_graph_feature.h"
#include "utils/visibility_graph_types.h"
#include "utils/ray_tracing.h"
#include <boost/graph/adjacency_list.hpp>
#include <vector>

namespace vulcan
{
namespace utils
{

struct VisibilityGraphVertex
{
    VisGraphVertex position;
};

struct VisibilityGraphEdge
{
    float distance;
};

using VisGraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VisibilityGraphVertex, VisibilityGraphEdge>;


/**
* VisibilityGraph implements algorithms to construct, search, and access properties of a visibility graph.
*
* A visibility graph consists of a set of vertices and edge. Each vertex is connected to other vertices that
* are visible from it. In the context of this VisibilityGraph implementation, a vertex B is visible from vertex A
* if a ray from the vertex A can be traced through the grid to vertex B without colliding with an obstacle. See
* the constructor for particulars on how this behavior is implemented.
*/
class VisibilityGraph
{
public:

    using Edge = std::pair<VisGraphVertex, VisGraphVertex>;

    /**
    * Default constructor for VisibilityGraph.
    *
    * Create an empty visibility graph.
    */
    VisibilityGraph(void) { }

    /**
    * Constructor for VisibilityGraph.
    *
    * Create a VisibilityGraph from a collection of points. Each pair of points is tested for visibility within the graph.
    * If the points are visible, an edge is added between them.
    */
    template <class VertexIterator, class Grid, class TerminationFunc>
    VisibilityGraph(VertexIterator begin,
                    VertexIterator end,
                    double maxDistance,     // maximum distance (in cells) to consider visibility
                    const Grid& grid,
                    TerminationFunc termination)
    : graph_(std::distance(begin, end))
    , vertices_(begin, end)
    {
        int numVertices = std::distance(begin, end);

        for(int n = 0; n < numVertices; ++n)
        {
            for(int i = n+1; i < numVertices; ++i)
            {
                double distance = distance_between_points(vertices_[n], vertices_[i]);

                if((distance < maxDistance) &&
                    is_cell_visible_from_position(vertices_[n], vertices_[i], grid, termination))
                {
                    auto edge = boost::add_edge(n, i, graph_);
                    if(edge.second)
                    {
                        graph_[edge.first].distance = distance_between_points(vertices_[n], vertices_[i]);
                    }

                    edges_.push_back(std::make_pair(vertices_[n], vertices_[i]));
                }
            }

            graph_[n].position = vertices_[n];
            descriptors_[vertices_[n]] = n;
        }
    }

    /**
    * Constructor for VisibilityGraph.
    *
    * Create a VisibilityGraph from a known set of points and edges. Mostly intended for testing purposes.
    */
    VisibilityGraph(const std::vector<VisGraphVertex>& points, const std::vector<std::pair<int, int>>& edges);

    /**
    * calculateFeature calculates a particular feature for the visibility graph. These features are defined in
    * visibility_graph_feature.h.
    *
    * \param    type            Type of feature to be calculated
    */
    VisibilityGraphFeature calculateFeature(VisibilityGraphFeatureType type) const;

    /**
    * isVertex queries if a particular vertex is in the visibility graph.
    */
    bool isVertex(VisGraphVertex vertex) const;

    /**
    * createSubgraph creates a subgraph containing only the provided vertices and any edges joining them.
    *
    * \param    begin           Start of the range of vertices
    * \param    end             End of the range of vertices
    * \return   An induced subgraph containing only the vertices and any edges connecting them within this visiblity
    *           graph.
    */
    VisibilityGraph createSubgraph(VisVertexIter begin, VisVertexIter end) const;

    /**
    * edges retrieves all edges in the VisibilityGraph.
    */
    std::vector<Edge> edges(void) const { return edges_; }

    /**
    * vertices retrieves all vertices in the VisibilityGraph.
    */
    std::vector<VisGraphVertex> vertices(void) const { return vertices_; }

private:

    using VertexToDesc = PointToTypeMap<int, VisGraphType::vertex_descriptor>;

    VisGraphType graph_;
    VertexToDesc descriptors_;
    std::vector<VisGraphVertex> vertices_;
    std::vector<Edge> edges_;

    mutable std::vector<VisibilityGraphFeature> features_;

    void calculateFeatures(void) const;
    int findCachedFeatureIndex(VisibilityGraphFeatureType type) const;
    VisibilityGraphFeature meanPathCount(void) const;
    VisibilityGraphFeature clusteringCoeff(void) const;
    VisibilityGraphFeature degreeCentrality(void) const;
    VisibilityGraphFeature closenessCentrality(void) const;
    VisibilityGraphFeature betweennessCentrality(void) const;
    VisibilityGraphFeature pagerank(void) const;
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_VISIBILITY_GRAPH_H
