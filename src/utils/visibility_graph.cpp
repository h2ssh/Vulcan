/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph.cpp
* \author   Collin Johnson
* 
* Definition of VisibilityGraph.
*/

#include "utils/visibility_graph.h"
#include "utils/algorithm_ext.h"
#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/closeness_centrality.hpp>
#include <boost/graph/clustering_coefficient.hpp>
#include <boost/graph/degree_centrality.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/page_rank.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <algorithm>
#include <iostream>
#include <unordered_set>

namespace vulcan
{
namespace utils
{

using namespace boost;

// Define property map for storing the distances between nodes
using DistProperty = exterior_vertex_property<VisGraphType, int>;
using DistVec = DistProperty::container_type;
using DistMatrix = DistProperty::matrix_type;
using DistMap = DistProperty::matrix_map_type;

// Define a property map for storing the calculated feature values
using FeatureProperty = exterior_vertex_property<VisGraphType, float>;
using FeatureContainer = FeatureProperty::container_type;
using FeatureMap = FeatureProperty::map_type;


// Types for creating subgraphs easily
using VertexSet = PointSet<int>;

struct VertexSetVertex
{
    const VertexSet* idSet;
    const VisGraphType* graph;
    
    bool operator()(VisGraphType::vertex_descriptor vertex) const
    {
        return idSet->find((*graph)[vertex].position) != idSet->end();
    }
    
    explicit VertexSetVertex(const VertexSet* idSet= nullptr,
                             const VisGraphType* graph = nullptr)
    : idSet(idSet)
    , graph(graph)
    {
    }
};

struct VertexSetEdge
{
    const VertexSet* idSet;
    const VisGraphType* graph;
    
    bool operator()(VisGraphType::edge_descriptor edge) const
    {
        return (idSet->find((*graph)[source(edge, *graph)].position) != idSet->end())
            && (idSet->find((*graph)[target(edge, *graph)].position) != idSet->end());
    }
    
    explicit VertexSetEdge(const VertexSet* idSet= nullptr,
                           const VisGraphType* graph = nullptr)
    : idSet(idSet)
    , graph(graph)
    {
    }
};


void all_pairs_paths(const VisGraphType& graph, DistMap& distanceMap);
VisibilityGraphFeature feature_map_to_graph_feature(const FeatureMap& featMap,
                                                    const VisGraphType& graph,
                                                    VisibilityGraphFeatureType type);

std::tuple<double, double> edge_count_stats(const DistMatrix& distMap, int numVertices);


/////////////////// VisibilityGraph implementation //////////////////////////

VisibilityGraph::VisibilityGraph(const std::vector<VisGraphVertex>& points, const std::vector<std::pair<int, int>>& edges)
: graph_(points.size())
, vertices_(points)
{
    for(std::size_t n = 0; n < points.size(); ++n)
    {
        graph_[n].position = points[n];
    }

    for(auto& edge : edges)
    {
        auto desc = add_edge(edge.first, edge.second, graph_);
        if(desc.second)
        {
            graph_[desc.first].distance = distance_between_points(vertices_[edge.first], vertices_[edge.second]);
            edges_.push_back(std::make_pair(vertices_[edge.first], vertices_[edge.second]));
        }
    }
}


VisibilityGraphFeature VisibilityGraph::calculateFeature(VisibilityGraphFeatureType type) const
{
    int cachedFeatureIndex = findCachedFeatureIndex(type);
    if(cachedFeatureIndex != -1)
    {
        return features_[cachedFeatureIndex];
    }

    VisibilityGraphFeature feature;

    switch(type)
    {
    case VisibilityGraphFeatureType::mean_edge_count:
        feature = meanPathCount();
        break;

    case VisibilityGraphFeatureType::clustering_coeff:
        feature = clusteringCoeff();
        break;

    case VisibilityGraphFeatureType::degree_centrality:
        feature = degreeCentrality();
        break;

    case VisibilityGraphFeatureType::closeness_centrality:
        feature = closenessCentrality();
        break;

    case VisibilityGraphFeatureType::betweenness_centrality:
        feature = betweennessCentrality();
        break;
        
    case VisibilityGraphFeatureType::pagerank:
        feature = pagerank();
        break;

    case VisibilityGraphFeatureType::none:
    case VisibilityGraphFeatureType::num_features:
    default:
        break;
    }

    if(feature.type() != VisibilityGraphFeatureType::none)
    {
        features_.emplace_back(feature);
    }

    return feature;
}


bool VisibilityGraph::isVertex(VisGraphVertex vertex) const
{
    return contains(vertices_, vertex);
}


VisibilityGraph VisibilityGraph::createSubgraph(VisVertexIter begin, VisVertexIter end) const
{
    VertexSet subVerts(begin, end);
    
    for(auto v : make_iterator_range(begin, end))
    {
        for(auto adj : make_iterator_range(adjacent_vertices(descriptors_.at(v), graph_)))
        {
            subVerts.insert(graph_[adj].position);
        }
    }
    
    VertexSetVertex vertexFilter(&subVerts, &graph_);
    VertexSetEdge edgeFilter(&subVerts, &graph_);
    auto filtered = make_filtered_graph(graph_, edgeFilter, vertexFilter);
    
    std::unordered_map<VisGraphType::vertex_descriptor, std::size_t> oldToNew;
    
    // Copy the filtered graph into a new graph
    VisibilityGraph subgraph;
    
    // Copy over the vertices that exist in the new subgraph
    for(auto v : make_iterator_range(boost::vertices(filtered)))
    {
        oldToNew[v] = add_vertex(graph_[v], subgraph.graph_);
        subgraph.vertices_.push_back(graph_[v].position);
    }
    
    // Copy over the edges that exist in the new subgraph
    for(auto e : make_iterator_range(boost::edges(filtered)))
    {
        add_edge(oldToNew[source(e, graph_)], oldToNew[target(e, graph_)], subgraph.graph_);
        subgraph.edges_.push_back(std::make_pair(graph_[source(e, graph_)].position,
                                                 graph_[target(e, graph_)].position));
    }
    
    return subgraph;
}


int VisibilityGraph::findCachedFeatureIndex(VisibilityGraphFeatureType type) const
{
    for(std::size_t n = 0; n < features_.size(); ++n)
    {
        if(features_[n].type() == type)
        {
            return n;
        }
    }

    return -1;
}


VisibilityGraphFeature VisibilityGraph::meanPathCount(void) const
{
    int numVertices = num_vertices(graph_);

    // Compute the distances between all pairs of vertices
    DistMatrix distances(numVertices);
    DistMap distMap(distances, graph_);
    all_pairs_paths(graph_, distMap);

    FeatureContainer pathDists(numVertices);
    FeatureMap pathDistMap(pathDists, graph_);

    for(int n = 0; n < numVertices; ++n)
    {
        double sum = 0.0;

        for(auto& pathLength : distances[n])
        {
            // Only care about paths leading to other areas
            if((pathLength < numVertices) && (pathLength > 0))
            {
                sum += pathLength;
            }
        }

        pathDistMap[n] = (numVertices > 1) ? sum / (numVertices - 1) : 0.0f;
    }

    double maxMean = *std::max_element(pathDists.begin(), pathDists.end());
    if(maxMean > 0.0)
    {
        for(auto& dist : pathDists)
        {
            dist /= maxMean;
        }
    }

    return feature_map_to_graph_feature(pathDistMap, graph_, VisibilityGraphFeatureType::mean_edge_count);
}


VisibilityGraphFeature VisibilityGraph::clusteringCoeff(void) const
{
    // Compute the degree centrality for graph.
    FeatureContainer coeffs(num_vertices(graph_));
    FeatureMap coeffMap(coeffs, graph_);
    all_clustering_coefficients(graph_, coeffMap);

    return feature_map_to_graph_feature(coeffMap, graph_, VisibilityGraphFeatureType::clustering_coeff);
}


VisibilityGraphFeature VisibilityGraph::degreeCentrality(void) const
{
    // Compute the degree centrality for graph.
    FeatureContainer centralities(num_vertices(graph_));
    FeatureMap centMap(centralities, graph_);
    all_degree_centralities(graph_, centMap);

    double maxDegree = *std::max_element(centralities.begin(), centralities.end());
    // Normalize the degree centrality by the maximum possible value (num verts - 1)
    if(maxDegree > 0.0)
    {
        for(auto& c : centralities)
        {
            c /= maxDegree;
        }
    }

    return feature_map_to_graph_feature(centMap, graph_, VisibilityGraphFeatureType::degree_centrality);
}


VisibilityGraphFeature VisibilityGraph::closenessCentrality(void) const
{
    // Compute the distances between all pairs of vertices
    DistMatrix distances(num_vertices(graph_));
    DistMap distMap(distances, graph_);
    all_pairs_paths(graph_, distMap);

    // Compute the closeness centrality for graph.
    FeatureContainer centralities(num_vertices(graph_));
    FeatureMap centMap(centralities, graph_);
    all_closeness_centralities(graph_, distMap, centMap);

    // Normalize the closeness to have a value of 1 if a node is connected to every other node and decrease
    // from there. The value returned here is 1 / sum(path dists). If fully connected, this sum is (num verts - 1),
    // so just multiply by that to get the normalized value here
    if(num_vertices(graph_) > 1)
    {
        double normalizer = num_vertices(graph_) - 1.0;

        for(auto& c : centralities)
        {
            c *= normalizer;
        }
    }

    return feature_map_to_graph_feature(centMap, graph_, VisibilityGraphFeatureType::closeness_centrality);
}


VisibilityGraphFeature VisibilityGraph::betweennessCentrality(void) const
{
    // Compute the betweenness centrality for graph.
    FeatureContainer centralities(num_vertices(graph_));
    FeatureMap centMap(centralities, graph_);
    brandes_betweenness_centrality(graph_, centMap);

    // Normalize the betweenness values
    if(num_vertices(graph_) > 1)
    {
        double min = *std::min_element(centralities.begin(), centralities.end());
        double max = *std::max_element(centralities.begin(), centralities.end());

        // Scale the betweenness relative to the min and max to make it a relative measure.
        double normalizer = (max > min) ? max - min : 1.0;

        for(auto& c : centralities)
        {
            c = (c - min) / normalizer;
        }
    }

    return feature_map_to_graph_feature(centMap, graph_, VisibilityGraphFeatureType::betweenness_centrality);
}


VisibilityGraphFeature VisibilityGraph::pagerank(void) const
{
    // Compute the betweenness centrality for graph.
    FeatureContainer ranks(num_vertices(graph_));
    FeatureMap rankMap(ranks, graph_);
    page_rank(graph_, rankMap);

    return feature_map_to_graph_feature(rankMap, graph_, VisibilityGraphFeatureType::pagerank);
}


void all_pairs_paths(const VisGraphType& graph, DistMap& distanceMap)
{
    auto uniformCostMap = static_property_map<int>(1);
    johnson_all_pairs_shortest_paths(graph, distanceMap, weight_map(uniformCostMap));
}


VisibilityGraphFeature feature_map_to_graph_feature(const FeatureMap& featMap,
                                                    const VisGraphType& graph,
                                                    VisibilityGraphFeatureType type)
{
    std::vector<VisibilityGraphFeature::value_type> values(num_vertices(graph));
    for(int n = 0, end = num_vertices(graph); n < end; ++n)
    {
        values[n] = std::make_pair(graph[n].position, featMap[n]);
    }

    return VisibilityGraphFeature(type, values);
}


std::tuple<double, double> edge_count_stats(const DistMatrix& distMap, int numVertices)
{
    using namespace accumulators;
    accumulator_set<double, stats<tag::mean, tag::variance>> statsAcc;
    
    for(int n = 0; n < numVertices; ++n)
    {
        for(auto& pathLength : distMap[n])
        {
            // Only care about paths leading to other areas
            if((pathLength < numVertices) && (pathLength > 0))
            {
                statsAcc(pathLength);
            }
        }
    }
    
    return std::make_pair(mean(statsAcc), std::sqrt(variance(statsAcc)));
}

} // namespace utils
} // namespace vulcan
