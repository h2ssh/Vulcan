/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     em.cpp
 * \author   Collin Johnson
 *
 * Definition of em_1d_linear and em_2d_fixed.
 */

#include <math/clustering.h>
#include <core/multivariate_gaussian.h>
#include <math/statistics.h>
#include <math/univariate_gaussian.h>
#include <boost/range/iterator_range.hpp>
#include <algorithm>
#include <iomanip>
#include <iostream>

namespace vulcan
{
namespace math
{

template <class Data>
using DataIt = typename std::vector<Data>::const_iterator;

template<class Data, class Dist>
struct em_data_t
{
    int numClusters;
    std::vector<int> clusterSizes;
    std::vector<Dist> dists;
    std::vector<int> assignments;
    std::vector<Data> clusterData;  // temporary storage for computing distributions
    double totalProb;
};

template <class Data, class Dist, class ProbFunc, class Estimator>
em_data_t<Data, Dist> run_em(DataIt<Data> begin,
                             DataIt<Data> end,
                             int k,
                             int maxIterations,
                             ProbFunc prob,
                             Estimator estimator);

template <class Data, class Dist, class Estimator>
void initialize_dists(DataIt<Data> begin, DataIt<Data> end, int k, em_data_t<Data, Dist>& dists, Estimator estimator);

template <class Data, class Dist, class Estimator>
void calculate_dists(DataIt<Data> begin, DataIt<Data> end, em_data_t<Data, Dist>& dists, Estimator estimator);

template <class Data, class Dist, class ProbFunc>
int assign_dists(DataIt<Data> begin, DataIt<Data> end, em_data_t<Data, Dist>& dists, ProbFunc prob);

template <class Data, class Dist, class ProbFunc>
int most_probable_cluster(Data data, const em_data_t<Data, Dist>& dists, ProbFunc prob);

template <class Data, class Dist>
void print_dists(const em_data_t<Data, Dist>& dists);

struct UnivariateEstimator
{
    UnivariateGaussianDistribution operator()(DataIt<double> begin, DataIt<double> end)
    {
        double var = variance(begin, end);
        return UnivariateGaussianDistribution(mean(begin, end), std::max(var, 1e-10));
    }
};

struct MultivariateEstimator
{
    MultivariateGaussian operator()(DataIt<Point<float>> begin, DataIt<Point<float>> end)
    {
        Vector mean(2);
        mean.zeros();
        for(auto p : boost::make_iterator_range(begin, end))
        {
            mean[0] += p.x;
            mean[1] += p.y;
        }

        if(std::distance(begin, end) > 0)
        {
            mean /= std::distance(begin, end);
        }

        Matrix cov(2, 2);
        cov.zeros();
        Vector diff(2);
        for(auto p : boost::make_iterator_range(begin, end))
        {
            diff[0] = p.x - mean[0];
            diff[1] = p.y - mean[1];

            cov += diff * arma::trans(diff);
        }

        if(std::distance(begin, end) < 2)
        {
            cov.zeros();
            cov.diag().fill(1e-16);
        }
        else
        {
            cov /= std::distance(begin, end) - 1;
        }

        try
        {
            Matrix inv = arma::inv(cov);
        }
        catch(std::exception& e)
        {
            std::cerr << "Failed to take inverse: " << e.what() << " Matrix:\n" << cov;
            cov.zeros();
            cov.diag().fill(1e-16);
        }

        return MultivariateGaussian(mean, cov);
    }
};


inline double probability_univariate(double value, const UnivariateGaussianDistribution& mean)
{
    return mean.likelihood(value);
}

inline double probability_multivariate(const Point<float>& value , const MultivariateGaussian& mean)
{
    Vector vec(2);
    vec[0] = value.x;
    vec[1] = value.y;
    return mean.probability(vec);
}


clustering_result_t em_1d_linear(std::vector<double>::const_iterator begin,
                                 std::vector<double>::const_iterator end,
                                 int kMax,
                                 int maxIterations)
{
    int dataSize = std::distance(begin, end);
    kMax = std::min(kMax, dataSize);    // can't have more clusters than data!

    std::vector<em_data_t<double, UnivariateGaussianDistribution>> attemptedMeans;
    for(int k = 1; k <= kMax; ++k)
    {
        attemptedMeans.push_back(run_em<double, UnivariateGaussianDistribution>(begin,
                                                                                end,
                                                                                k,
                                                                                maxIterations,
                                                                                probability_univariate,
                                                                                UnivariateEstimator()));
    }

    auto maxProbIt = std::max_element(attemptedMeans.begin(), attemptedMeans.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.totalProb < rhs.totalProb;
    });

    // Convert the em_data_t to a clustering_result_t
    clustering_result_t results;
    results.numClusters = maxProbIt->dists.size();
    results.clusterSizes = std::move(maxProbIt->clusterSizes);
    results.assignedCluster = std::move(maxProbIt->assignments);

#ifdef DEBUG_RESULTS
    std::cout << "INFO: em_1d_linear: Cluster results:\nNum clusters:" << results.numClusters << " Cluster sizes:\n";
    for(std::size_t n = 0; n < minMeanIt->dists.size(); ++n)
    {
        std::cout << std::setprecision(5) << std::setw(5) << minMeanIt->dists[n] << "->" << results.clusterSizes[n] << '\n';
    }
#endif

    return results;
}


clustering_result_t em_2d_linear(std::vector<Point<float>>::const_iterator begin,
                                 std::vector<Point<float>>::const_iterator end,
                                 const int kMax,
                                 const int maxIterations)
{
    std::vector<em_data_t<Point<float>, MultivariateGaussian>> clusters;

    for(int k = 1; k < kMax; ++k)
    {
        clusters.push_back(run_em<Point<float>, MultivariateGaussian>(begin,
                                                                      end,
                                                                      k,
                                                                      maxIterations,
                                                                      probability_multivariate,
                                                                      MultivariateEstimator()));
    }

    auto maxProbIt = std::max_element(clusters.begin(), clusters.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.totalProb < rhs.totalProb;
    });

    // Convert the em_data_t to a clustering_result_t
    clustering_result_t results;
    results.numClusters = maxProbIt->dists.size();
    results.clusterSizes = std::move(maxProbIt->clusterSizes);
    results.assignedCluster = std::move(maxProbIt->assignments);

#ifdef DEBUG_RESULTS
    std::cout << "INFO: em_2d_fixed: Cluster results:\nNum clusters:" << results.numClusters << " Cluster sizes:\n";
    for(std::size_t n = 0; n < dists.dists.size(); ++n)
    {
        std::cout << std::setprecision(5) << std::setw(5) << dists.dists[n] << "->" << results.clusterSizes[n] << '\n';
    }
#endif

    return results;
}


template <class Data, class Dist, class ProbFunc, class Estimator>
em_data_t<Data, Dist> run_em(DataIt<Data> begin,
                             DataIt<Data> end,
                             int k,
                             int maxIterations,
                             ProbFunc prob,
                             Estimator estimator)
{
    em_data_t<Data, Dist> dists;
    dists.numClusters = k;
    initialize_dists(begin, end, k, dists, estimator);
    calculate_dists(begin, end, dists, estimator);

    int numChanges = 0;
    int numIterations = 0;

    do
    {
        numChanges = assign_dists(begin, end, dists, prob);
        calculate_dists(begin, end, dists, estimator);
        ++numIterations;

#ifdef DEBUG_KMEANS
        std::cout << "Iteration:" << numIterations << " Changes:" << numChanges << '\n';
        print_dists(dists);
#endif

    } while((numChanges > 0) && (numIterations <= maxIterations));

    // Once complete, sum the error amongst all the dists
    for(std::size_t n = 0, size = std::distance(begin, end); n < size; ++n)
    {
        dists.totalProb += prob(*(begin + n), dists.dists[dists.assignments[n]]);
    }

    // If any cluster is empty, zero probability of correctness
    for(std::size_t n = 0; n < dists.clusterSizes.size(); ++n)
    {
        if(dists.clusterSizes[n] == 0)
        {
            dists.totalProb = 0.0;
        }
    }

#ifdef DEBUG_KMEANS
    std::cout << "INFO: em_1d: k:" << k << " Error:" << dists.totalProb << " Num iterations:" << numIterations
        << '\n';
    print_dists(dists);
#endif

    return dists;
}


template <class Data, class Dist, class Estimator>
void initialize_dists(DataIt<Data> begin, DataIt<Data> end, int k, em_data_t<Data, Dist>& dists, Estimator estimator)
{
    // Allocate the buffers
    dists.clusterSizes.resize(k);
    dists.dists.resize(k);
    dists.assignments.resize(std::distance(begin, end));
    std::iota(dists.assignments.begin(), dists.assignments.end(), 0);
    dists.totalProb = 0.0;

    std::transform(dists.assignments.begin(), dists.assignments.end(), dists.assignments.begin(), [k](int c) {
        return c % k;
    });
}


template <class Data, class Dist, class Estimator>
void calculate_dists(DataIt<Data> begin, DataIt<Data> end, em_data_t<Data, Dist>& dists, Estimator estimator)
{
    for(int cluster = 0; cluster < dists.numClusters; ++cluster)
    {
        dists.clusterData.clear();

        for(std::size_t n = 0; n < dists.assignments.size(); ++n)
        {
            if(dists.assignments[n] == cluster)
            {
                dists.clusterData.push_back(*(begin + n));
            }
        }

        dists.dists[cluster] = estimator(dists.clusterData.begin(), dists.clusterData.end());
    }
}


template <class Data, class Dist, class ProbFunc>
int assign_dists(DataIt<Data> begin, DataIt<Data> end, em_data_t<Data, Dist>& dists, ProbFunc prob)
{
    std::fill(dists.clusterSizes.begin(), dists.clusterSizes.end(), 0);

    int numChanged = 0;

    for(std::size_t n = 0, size = std::distance(begin, end); n < size; ++n)
    {
        int newAssignment = most_probable_cluster(*(begin + n), dists, prob);
        if(newAssignment != dists.assignments[n])
        {
            dists.assignments[n] = newAssignment;
            ++numChanged;
        }

        ++dists.clusterSizes[newAssignment];
    }

    return numChanged;
}


template <class Data, class Dist, class ProbFunc>
int most_probable_cluster(Data data, const em_data_t<Data, Dist>& dists, ProbFunc prob)
{
    auto maxIt = std::max_element(dists.dists.begin(), dists.dists.end(), [&](auto& lhs, auto& rhs) {
        return prob(data, lhs) < prob(data, rhs);
    });

    return std::distance(dists.dists.begin(), maxIt);
}


template <class Data, class Dist>
void print_dists(const em_data_t<Data, Dist>& dists)
{
    for(std::size_t n = 0; n < dists.dists.size(); ++n)
    {
        std::cout << std::setprecision(5) << std::setw(5) << dists.dists[n] << "->" << dists.clusterSizes[n] << '\n';
    }
}

} // namespace utils
} // namespace vulcan
