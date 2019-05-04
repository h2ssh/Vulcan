/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph_feature.h
* \author   Collin Johnson
*
* Declaration of VisibilityGraphFeature.
*/

#ifndef UTILS_VISIBILITY_GRAPH_FEATURE_H
#define UTILS_VISIBILITY_GRAPH_FEATURE_H

#include <utils/visibility_graph_types.h>
#include <string>
#include <utility>

namespace vulcan
{
namespace utils
{

/**
* VisibilityGraphFeatureType defines an identifier for all possible feature types for the visibility graph.
*/
enum class VisibilityGraphFeatureType
{
    none,
    mean_edge_count,                ///< Mean number of edges in paths to all other nodes in the graph
    clustering_coeff,               ///< Clustering coefficient (how connected neighbors are)
    degree_centrality,              ///< Degree of vert / num possible edges
    closeness_centrality,           ///< Min possible sum path lengths / sum of path lengths
    betweenness_centrality,         ///< Number shortest paths traveling through the vertex
    pagerank,                       ///< PageRank of the particular node -- variant of eigenvector centrality
    num_features,
};

std::string feature_type_to_string(VisibilityGraphFeatureType type);


/**
* feature_stats_t contains all the statistics calculated for a given feature.
*/
struct feature_stats_t
{
    double min;
    double mean;
    double max;
    double std;
};

/**
* VisibilityGraphFeature provides a simple abstraction for a feature in the visiblity graph. Providing access to
* the value for each vertex in the graph along with statistics for the whole graph or a subset of vertices in
* the graph.
*/
class VisibilityGraphFeature
{
public:

    using value_type = std::pair<VisGraphVertex, double>;
    using const_iterator = std::vector<value_type>::const_iterator;


    /**
    * Default constructor for VisibilityGraphFeature.
    *
    * Creates an empty collection of features with type == none.
    */
    VisibilityGraphFeature(void);

    /**
    * Constructor for VisibilityGraphFeature.
    *
    * \param    type            Type of the features
    * \param    values          Values for the particular feature
    */
    VisibilityGraphFeature(VisibilityGraphFeatureType type, const std::vector<value_type>& values);

    /**
    * type retrieves the type of the feature.
    */
    VisibilityGraphFeatureType type(void) const { return type_; }

    /**
    * stats calculates the mean and standard deviation of the feature value for the provided vertices. Pass an empty
    * range (begin == end) to calculate for all values. Not all of the passed in vertices must exist in the graph. If
    * the given vertex doesn't exist, then it will just be ignored in the final calculation.
    *
    * \param    vertices        Vertices with which to calculate the mean
    * \return   Statistics of the feature for those vertices in the graph.
    */
    feature_stats_t stats(std::vector<VisGraphVertex>::const_iterator begin,
                          std::vector<VisGraphVertex>::const_iterator end) const;

    /**
    * begin retrieves the beginning iterator to the vertex, value pairs for the feature.
    */
    const_iterator begin(void) const { return values_.begin(); }

    /**
    * end retrieves the one-past-the-end iterator for the vertex, value pairs.
    */
    const_iterator end(void) const { return values_.end(); }

private:

    VisibilityGraphFeatureType type_;
    std::vector<value_type> values_;
};

}
}

#endif // UTILS_VISIBILITY_GRAPH_FEATURE_H
