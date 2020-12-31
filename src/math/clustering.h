/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     clustering.h
 * \author   Collin Johnson
 *
 * Declaration of functions implementing the DBScan algorithm:
 *
 *   - dbscan_1d_linear : cluster 1d data whose distance function is linear, i.e. absolute value of difference
 *   - kmeans_1d_linear : k-means clustering for 1d data using squared error
 *   - kmeans_2d_fixed  : k-means clustering for 2d points using squared error and a fixed number of means
 */

#ifndef MATH_CLUSTERING_H
#define MATH_CLUSTERING_H

#include "core/point.h"
#include <vector>

namespace vulcan
{
namespace math
{

const int kNoiseCluster = -1;   ///< The cluster assigned to points identified as noise

/**
 * clustering_result_t contains the results of running a clustering algorithm on a collection of data. The results
 * contain the cluster for each input point and the number of clusters.
 */
struct clustering_result_t
{
    int numClusters;                    ///< Number of clusters found in the data
    std::vector<int> clusterSizes;      ///< Information on the size of each found cluster
    std::vector<int> assignedCluster;   ///< The cluster index assigned to each point in the input data. The size
                                        ///< is the same as the input and the index is the same, i.e. assignedCluster[0]
                                        ///< is the cluster assigned to the data point at data.begin.
};

/**
 * dbscan_1d_linear runs the DBScan algorithm to cluster the input data, which is simple one-dimensional data with a
 * linear distance function that is the absolute value of the difference.
 *
 * The two parameters for DBScan are:
 *
 *   - epsilon: maximum distance between two measurements for a measurement to be in the same neighborhood
 *   - minPoints: minimum number of points that must be in a points neighborhood for it to not be noise
 *
 * \param    begin           Start of the data
 * \param    end             One-past-the-end of the data
 * \param    epsilon         Epsilon value to use for determining the size of a point's neighborhood
 * \param    minPoints       Minimum size of a neighbor for non-noise points
 * \return   A clustering_result_t with the clustering results for the provided data.
 */
clustering_result_t dbscan_1d_linear(std::vector<double>::const_iterator begin,
                                     std::vector<double>::const_iterator end,
                                     double epsilon,
                                     int minPoints);

/**
 * kmeans_1d_linear runs k-means clustering to cluster the input data. The search will search from 1 to kMax clusters
 * and select the result that minimizes the total error.
 *
 * \param    begin           Start of the data
 * \param    end             One-past-the-end of the data
 * \param    kMax            Maximum number of clusters to consider
 * \param    maxIterations   Maximum number of iterations to run (optional, default = 20)
 * \return   A clustering_result_t
 */
clustering_result_t kmeans_1d_linear(std::vector<double>::const_iterator begin,
                                     std::vector<double>::const_iterator end,
                                     int kMax,
                                     int maxIterations = 20);

/**
 * kmeans_2d_fixed runs k-means clustering to cluster the input data. k-means will be run with a single, fixed number
 * of clusters.
 *
 * \param    begin           Start of the data
 * \param    end             One-past-the-end of the data
 * \param    k               Number of clusters to consider
 * \param    maxIterations   Maximum number of iterations to run (optional, default = 20)
 * \return   A clustering_result_t
 */
clustering_result_t kmeans_2d_fixed(std::vector<Point<float>>::const_iterator begin,
                                    std::vector<Point<float>>::const_iterator end,
                                    int k,
                                    int maxIterations = 20);

/**
 * em_1d_linear runs the Expectation-Maximization algorithm to cluster the input data. The search will search from 1 to
 * kMax clusters and select the result that minimizes the total error.
 *
 * \param    begin           Start of the data
 * \param    end             One-past-the-end of the data
 * \param    kMax            Maximum number of clusters to consider
 * \param    maxIterations   Maximum number of iterations to run (optional, default = 20)
 * \return   A clustering_result_t
 */
clustering_result_t em_1d_linear(std::vector<double>::const_iterator begin,
                                 std::vector<double>::const_iterator end,
                                 int kMax,
                                 int maxIterations = 20);

/**
 * kmeans_2d_fixed runs the Expectation-Maximization algorithm to cluster the input data.
 *
 * \param    begin           Start of the data
 * \param    end             One-past-the-end of the data
 * \param    k               Number of clusters to consider
 * \param    maxIterations   Maximum number of iterations to run (optional, default = 20)
 * \return   A clustering_result_t
 */
clustering_result_t em_2d_linear(std::vector<Point<float>>::const_iterator begin,
                                 std::vector<Point<float>>::const_iterator end,
                                 int kMax,
                                 int maxIterations = 20);
}   // namespace math
}   // namespace vulcan

#endif   // MATH_CLUSTERING_H
