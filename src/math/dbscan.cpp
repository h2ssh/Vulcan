/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dbscan.cpp
* \author   Collin Johnson
*
* Definition of dbscan_1d_linear.
*/

#include "math/clustering.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <numeric>

// #define DEBUG_NEIGHBORHOOD
// #define DEBUG_CLUSTER
// #define DEBUG_RESULTS

namespace vulcan
{
namespace math
{

using DataIt = std::vector<double>::const_iterator;
using IndexIt = std::vector<DataIt>::const_iterator;
using IndexRange = std::pair<IndexIt, IndexIt>;

clustering_result_t construct_results(DataIt begin, DataIt end);
IndexRange find_data_neighborhood(IndexIt data, IndexIt before, IndexIt after, IndexIt end, double epsilon);
IndexRange expand_cluster(IndexIt begin, IndexIt after, IndexIt end, double epsilon, int minPoints);
void add_cluster_to_results(IndexRange cluster, DataIt begin, clustering_result_t& results);
std::vector<double> calculate_cluster_means(DataIt begin, DataIt end, const clustering_result_t& results);

inline int range_size(IndexRange range) { return std::distance(range.first, range.second); }


clustering_result_t dbscan_1d_linear(DataIt begin, DataIt end, double epsilon, int minPoints)
{
    // Initialize the results by assigning all data to the noisy cluster.
    clustering_result_t results = construct_results(begin, end);

    // Sort the data by value in ascending order.
    std::vector<DataIt> indexes(std::distance(begin, end));
    std::iota(indexes.begin(), indexes.end(), begin);
    std::sort(indexes.begin(), indexes.end(), [](DataIt lhs, DataIt rhs) { return *lhs < *rhs; });

    /*
    * Until all data have been assigned a cluster:
    *   Find the neighborhood of the next data point to consider
    *   Grow the neighborhood fully
    *   If not large enough, assign as a noisy neighborhood
    *   Otherwise, assign the next cluster id
    *   Use the end of the previous neighborhood as the start of the next neighborhood
    */

    // The initial cluster has no data before or after
    IndexRange clusterEnds(indexes.begin(), indexes.begin());

    do
    {
        // The search for the new cluster begins from the end of the previous cluster
        clusterEnds = find_data_neighborhood(clusterEnds.second,
                                             clusterEnds.second,
                                             clusterEnds.second,
                                             indexes.end(),
                                             epsilon);
        clusterEnds = expand_cluster(clusterEnds.first, clusterEnds.second, indexes.end(), epsilon, minPoints);

        // If the cluster is large enough, add it to the results
        if(range_size(clusterEnds) >= minPoints)
        {
            add_cluster_to_results(clusterEnds, begin, results);
        }
        else // a cluster isn't added, so just start the search at the next point, which is ends.first + 1
        {
            clusterEnds.second = clusterEnds.first + 1;
        }
        // Otherwise, since all points are defaulted to noisy, ignore the results and keep searching
    } while(clusterEnds.second != indexes.end()); // Once the cluster end is the end of the indexes, no more clusters exist

#ifdef DEBUG_RESULTS
    auto means = calculate_cluster_means(begin, end, results);
    std::cout << "INFO: dbscan_1d_linear: Cluster results:\nNum clusters:" << results.numClusters << " Cluster sizes:\n";
    for(std::size_t n = 0; n < means.size(); ++n)
    {
        std::cout << std::setprecision(5) << std::setw(5) << means[n] << "->" << results.clusterSizes[n] << '\n';
    }
#endif

    return results;
}


clustering_result_t construct_results(DataIt begin, DataIt end)
{
    clustering_result_t results;
    results.numClusters = 0;
    results.assignedCluster.resize(std::distance(begin, end));

    // Initialize all data points as noise
    std::fill(results.assignedCluster.begin(), results.assignedCluster.end(), kNoiseCluster);

    return results;
}


IndexRange find_data_neighborhood(IndexIt data, IndexIt before, IndexIt after, IndexIt end, double epsilon)
{
    // The before point is the start of the neighbor. It contains the first point within epsilon of data
    before = std::find_if(before, data, [data, epsilon](DataIt point) {
        return std::abs(*point - **data) <= epsilon;
    });

    // The after point is the first point further than epsilon from data
    after = std::find_if(after, end, [data, epsilon](DataIt point) {
        return std::abs(*point - **data) > epsilon;
    });

#ifdef DEBUG_NEIGHBORHOOD
    std::cout << "INFO: dbscan_1d: Neighborhood for " << **data << " is:";
    for(auto it = before; it != after; ++it)
    {
        std::cout << **it << ' ';
    }
    std::cout << '\n';
#endif

    return std::make_pair(before, after);
}


IndexRange expand_cluster(IndexIt begin, IndexIt after, IndexIt end, double epsilon, int minPoints)
{
    // When expanding a cluster, the goal is to find the range of the cluster [begin, after). After is the one-past-the-end
    // iterator for the cluster.
    // For each point in the current range [current,after) create the neighborhood and extend the range of the after
    // iterator. If the neighborhood size is less than minPoints, then we have reached the end of the cluster. Alternately,
    // if the end of the cluster contains all points, then we've reached the end of the cluster too.

    IndexIt before = begin;
    IndexIt current = begin;

    while(current != end)
    {
        // Create the neighbor for the current point
        auto neighborhood = find_data_neighborhood(current, before, after, end, epsilon);

        // If the new neighborhood is large enough, extend the points in the cluster
        if(range_size(neighborhood) >= minPoints)
        {
            after = neighborhood.second; // Extended the neighborhood to include it
        }

        before = neighborhood.first;    // move the before marker to reduce the amount of search for the next point
        ++current; // move on to the next point in the range

        // Done if: or , then exit
        if((current == after)            // point is the last in the ordered neighborhood sequence
            || (after == end))           // the cluster contains all remaining data
        {
            break;
        }
    }

#ifdef DEBUG_CLUSTER
    std::cout << "INFO: dbscan_1d: Created cluster:";
    for(auto it = begin; it != after; ++it)
    {
        std::cout << **it << ' ';
    }
    std::cout << '\n';
#endif

    return std::make_pair(begin, after);
}


void add_cluster_to_results(IndexRange cluster, DataIt begin, clustering_result_t& results)
{
    int clusterId = results.numClusters;

    for(IndexIt index = cluster.first; index != cluster.second; ++index)
    {
        // The data point is the distance from the start of the input data
        int dataIndex = std::distance(begin, *index);
        results.assignedCluster[dataIndex] = clusterId;
    }

    results.clusterSizes.push_back(range_size(cluster));
    ++results.numClusters;
}


std::vector<double> calculate_cluster_means(DataIt begin, DataIt end, const clustering_result_t& results)
{
    std::vector<double> means(results.numClusters);
    std::fill(means.begin(), means.end(), 0.0);

    for(DataIt it = begin; it != end; ++it)
    {
        int index = results.assignedCluster[std::distance(begin, it)];
        if(index >= 0)
        {
            means[index] += (*it) / results.clusterSizes[index];
        }
    }

    return means;
}

} // namespace math
} // namespace vulcan
