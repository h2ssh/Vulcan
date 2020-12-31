/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     kmeans.cpp
 * \author   Collin Johnson
 *
 * Definition of kmeans_1d_linear.
 */

#include "math/clustering.h"
#include <algorithm>
#include <iomanip>
#include <iostream>

// #define DEBUG_KMEANS
// #define DEBUG_RESULTS

namespace vulcan
{
namespace math
{

template <class Data>
using DataIt = typename std::vector<Data>::const_iterator;

template <class Data>
struct kmeans_data_t
{
    std::vector<int> clusterSizes;
    std::vector<Data> means;
    std::vector<int> assignments;
    double totalError;
};

template <class Data, class DistFunc>
kmeans_data_t<Data> run_kmeans(DataIt<Data> begin, DataIt<Data> end, int k, int maxIterations, DistFunc dist);

template <class Data, class DistFunc>
void initialize_means(DataIt<Data> begin, DataIt<Data> end, int k, kmeans_data_t<Data>& means);

template <class Data>
void calculate_means(DataIt<Data> begin, DataIt<Data> end, kmeans_data_t<Data>& means);

template <class Data, class DistFunc>
int assign_means(DataIt<Data> begin, DataIt<Data> end, kmeans_data_t<Data>& means, DistFunc dist);

template <class Data, class DistFunc>
int closest_mean(Data data, const kmeans_data_t<Data>& means, DistFunc dist);

template <class Data>
void print_means(const kmeans_data_t<Data>& means);

// Define division internal to use with Point<float>
Point<float>& operator/=(Point<float>& p, int num)
{
    p.x /= num;
    p.y /= num;
    return p;
}

inline double distance(double value, double mean)
{
    return (value - mean) * (value - mean);
}


clustering_result_t kmeans_1d_linear(std::vector<double>::const_iterator begin,
                                     std::vector<double>::const_iterator end,
                                     int kMax,
                                     int maxIterations)
{
    int dataSize = std::distance(begin, end);
    kMax = std::min(kMax, dataSize);   // can't have more clusters than data!

    std::vector<kmeans_data_t<double>> attemptedMeans;
    for (int k = 1; k <= kMax; ++k) {
        attemptedMeans.push_back(run_kmeans<double>(begin, end, k, maxIterations, distance));
    }

    auto minMeanIt =
      std::min_element(attemptedMeans.begin(), attemptedMeans.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.totalError < rhs.totalError;
      });

    // Convert the kmeans_data_t to a clustering_result_t
    clustering_result_t results;
    results.numClusters = minMeanIt->means.size();
    results.clusterSizes = std::move(minMeanIt->clusterSizes);
    results.assignedCluster = std::move(minMeanIt->assignments);

#ifdef DEBUG_RESULTS
    std::cout << "INFO: kmeans_1d_linear: Cluster results:\nNum clusters:" << results.numClusters
              << " Cluster sizes:\n";
    for (std::size_t n = 0; n < minMeanIt->means.size(); ++n) {
        std::cout << std::setprecision(5) << std::setw(5) << minMeanIt->means[n] << "->" << results.clusterSizes[n]
                  << '\n';
    }
#endif

    return results;
}


clustering_result_t kmeans_2d_fixed(std::vector<Point<float>>::const_iterator begin,
                                    std::vector<Point<float>>::const_iterator end,
                                    const int k,
                                    const int maxIterations)
{
    auto means = run_kmeans<Point<float>>(begin, end, k, maxIterations, [](auto&& lhs, auto&& rhs) {
        return distance_between_points(lhs, rhs);
    });

    // Convert the kmeans_data_t to a clustering_result_t
    clustering_result_t results;
    results.numClusters = means.means.size();
    results.clusterSizes = std::move(means.clusterSizes);
    results.assignedCluster = std::move(means.assignments);

#ifdef DEBUG_RESULTS
    std::cout << "INFO: kmeans_2d_fixed: Cluster results:\nNum clusters:" << results.numClusters << " Cluster sizes:\n";
    for (std::size_t n = 0; n < means.means.size(); ++n) {
        std::cout << std::setprecision(5) << std::setw(5) << means.means[n] << "->" << results.clusterSizes[n] << '\n';
    }
#endif

    return results;
}


template <class Data, class DistFunc>
kmeans_data_t<Data> run_kmeans(DataIt<Data> begin, DataIt<Data> end, int k, int maxIterations, DistFunc dist)
{
    kmeans_data_t<Data> means;
    initialize_means(begin, end, k, means);

    int numChanges = 0;
    int numIterations = 0;

    do {
        numChanges = assign_means(begin, end, means, dist);
        calculate_means(begin, end, means);
        ++numIterations;
#ifdef DEBUG_KMEANS
        std::cout << "Iteration:" << numIterations << " Changes:" << numChanges << '\n';
        print_means(means);
#endif

    } while ((numChanges > 0) && (numIterations <= maxIterations));

    // Once complete, sum the error amongst all the means
    for (std::size_t n = 0, size = std::distance(begin, end); n < size; ++n) {
        means.totalError += dist(*(begin + n), means.means[means.assignments[n]]);
    }

    // If any cluster is empty, spike the error
    for (std::size_t n = 0; n < means.clusterSizes.size(); ++n) {
        if (means.clusterSizes[n] == 0) {
            means.totalError += 1000000.0;
        }
    }

#ifdef DEBUG_KMEANS
    std::cout << "INFO: kmeans_1d: k:" << k << " Error:" << means.totalError << " Num iterations:" << numIterations
              << '\n';
    print_means(means);
#endif

    return means;
}

template <class Data>
void initialize_means(DataIt<Data> begin, DataIt<Data> end, int k, kmeans_data_t<Data>& means)
{
    // Allocate the buffers
    means.clusterSizes.resize(k);
    means.means.resize(k);
    means.assignments.resize(std::distance(begin, end));
    std::fill(means.assignments.begin(), means.assignments.end(), -1);
    means.totalError = 0.0;

    // Assign values taken at a uniform increment through the data
    int increment = std::distance(begin, end) / k;

    for (int n = 0; n < k; ++n) {
        means.means[n] = *(begin + (increment * n));
    }
}

template <class Data>
void calculate_means(DataIt<Data> begin, DataIt<Data> end, kmeans_data_t<Data>& means)
{
    std::fill(means.means.begin(), means.means.end(), Data{});

    for (std::size_t n = 0, size = std::distance(begin, end); n < size; ++n) {
        means.means[means.assignments[n]] += *(begin + n);
    }

    for (std::size_t n = 0; n < means.means.size(); ++n) {
        if (means.clusterSizes[n] > 0) {
            means.means[n] /= means.clusterSizes[n];
        }
    }
}


template <class Data, class DistFunc>
int assign_means(DataIt<Data> begin, DataIt<Data> end, kmeans_data_t<Data>& means, DistFunc dist)
{
    std::fill(means.clusterSizes.begin(), means.clusterSizes.end(), 0);

    int numChanged = 0;

    for (std::size_t n = 0, size = std::distance(begin, end); n < size; ++n) {
        int newAssignment = closest_mean(*(begin + n), means, dist);
        if (newAssignment != means.assignments[n]) {
            means.assignments[n] = newAssignment;
            ++numChanged;
        }

        ++means.clusterSizes[newAssignment];
    }

    return numChanged;
}


template <class Data, class DistFunc>
int closest_mean(Data data, const kmeans_data_t<Data>& means, DistFunc dist)
{
    auto minIt = std::min_element(means.means.begin(), means.means.end(), [&](auto& lhs, auto& rhs) {
        return dist(data, lhs) < dist(data, rhs);
    });

    return std::distance(means.means.begin(), minIt);
}


template <class Data>
void print_means(const kmeans_data_t<Data>& means)
{
    for (std::size_t n = 0; n < means.means.size(); ++n) {
        std::cout << std::setprecision(5) << std::setw(5) << means.means[n] << "->" << means.clusterSizes[n] << '\n';
    }
}

}   // namespace math
}   // namespace vulcan
