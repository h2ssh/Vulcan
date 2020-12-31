/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     univariate_distribution_estimator_impl.cpp
 * \author   Collin Johnson
 *
 * Definition of implementations of the UnivariateDistributionEstimator for the subclasses of UnivariateDistribution:
 *
 *   - UnivariateGaussianDistribution
 *   - GammaDistribution
 *   - BetaDistribution
 *   - ExponentialDistribution
 *   - TruncatedGaussianDistribution
 */

#include "math/univariate_distribution_estimator_impl.h"
#include "math/beta_distribution.h"
#include "math/discrete_gaussian.h"
#include "math/exponential_distribution.h"
#include "math/gamma_distribution.h"
#include "math/statistics.h"
#include "math/truncated_gaussian_distribution.h"
#include "math/univariate_gaussian.h"
#include <boost/math/special_functions/digamma.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <iostream>

namespace vulcan
{
namespace math
{

using DistPtr = std::unique_ptr<UnivariateDistribution>;


double log_likelihood_gamma_func(double k, double sumXi, double sumLogXi, std::size_t n)
{
    return (k - 1) * sumXi - n * k - n * k * std::log(sumXi / (k * n)) - n * std::log(boost::math::tgamma(k));
}


double log_likelihood_gamma_deriv(double k, double sumXi, double sumLogXi, std::size_t n)
{
    return n * (std::log(k) - boost::math::digamma(k) - std::log(sumXi / n)) + sumLogXi;
}


DistPtr
  UnivariateGaussianDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new UnivariateGaussianDistribution(estimate(dataBegin, dataEnd))};
}


UnivariateGaussianDistribution
  UnivariateGaussianDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                                    const std::vector<double>::const_iterator dataEnd) const
{
    double var = variance(dataBegin, dataEnd);

    if (var == 0.0) {
        std::cerr << "WARNING: UnivariateGaussianDistributionEstimator: Variance in the data was 0. Setting variance "
                     "to 1e-4.\n";
        var = 1e-4;
    }

    return UnivariateGaussianDistribution(mean(dataBegin, dataEnd), var);
}


DistPtr
  DiscreteGaussianDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                              const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new DiscreteGaussianDistribution(estimate(dataBegin, dataEnd))};
}


DiscreteGaussianDistribution
  DiscreteGaussianDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                                  const std::vector<double>::const_iterator dataEnd) const
{
    double var = variance(dataBegin, dataEnd);

    if (var == 0.0) {
        std::cerr
          << "WARNING: DiscreteGaussianDistributionEstimator: Variance in the data was 0. Setting variance to 1e-4.\n";
        var = 1e-4;
    }

    return DiscreteGaussianDistribution(mean(dataBegin, dataEnd), var);
}


DistPtr GammaDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                         const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new GammaDistribution(estimate(dataBegin, dataEnd))};
}


GammaDistribution GammaDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                                       const std::vector<double>::const_iterator dataEnd) const
{
    std::vector<double> filtered(dataBegin, dataEnd);
    auto validDataEnd = std::remove_if(filtered.begin(), filtered.end(), [](double x) {
        return x <= 0.0;
    });

    int numValidData = std::distance(filtered.begin(), validDataEnd);
    assert(numValidData);

    using namespace std::placeholders;

    double sumXi = std::accumulate(filtered.begin(), validDataEnd, 0.0);
    double sumLogXi = 0.0;
    for (auto val : boost::make_iterator_range(filtered.begin(), validDataEnd)) {
        sumLogXi += std::log(val);
    }

    double s = std::log(sumXi / numValidData) - sumLogXi / numValidData;
    double k0 = (3.0 - s + std::sqrt(std::pow(s - 3.0, 2.0) + 24.0 * s)) / (12.0 * s);

    //     NewtonRaphsonErrorFunc<double> newton(std::bind(log_likelihood_func,  _1, sumXi, sumLogXi, filtered.size()),
    //                                           std::bind(log_likelihood_deriv, _1, sumXi, sumLogXi, filtered.size()));
    //
    //     double k     = find_single_root(newton, k0, 1e-5);
    double k = k0;
    double theta = sumXi / (numValidData * k);
    return GammaDistribution(k, theta);
}


DistPtr BetaDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                        const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new BetaDistribution(estimate(dataBegin, dataEnd))};
}


BetaDistribution BetaDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                                     const std::vector<double>::const_iterator dataEnd) const
{
    double mean = math::mean(dataBegin, dataEnd);
    double variance = math::variance(dataBegin, dataEnd);

    if (variance == 0.0) {
        std::cerr << "WARNING: BetaDistributionEstimator: Variance of data was 0. Setting to 1e-4.\n";
        variance = 1e-4;
    }

    if (variance < mean * (1 - mean) && (variance > 0.0)) {
        double alpha = mean * ((mean * (1.0 - mean) / variance) - 1.0);
        double beta = (1.0 - mean) * ((mean * (1.0 - mean) / variance) - 1.0);
        return BetaDistribution(alpha, beta);
    } else {
        std::cerr << "ERROR: BetaDistributionEstimator: Could not use method-of-moments to find parameters. Mean:"
                  << mean << " Variance:" << variance << '\n'
                  << " Variance should be less than " << (mean * (1 - mean)) << '\n';
        return BetaDistribution();
    }
}


ExponentialDistributionEstimator::ExponentialDistributionEstimator(double maxValue) : max_(maxValue)
{
}


DistPtr ExponentialDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                               const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new ExponentialDistribution(estimate(dataBegin, dataEnd))};
}


ExponentialDistribution
  ExponentialDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                             const std::vector<double>::const_iterator dataEnd) const
{
    double mean = math::mean(dataBegin, dataEnd);
    if (mean > 0.0) {
        return ExponentialDistribution(1.0 / mean, max_);
    } else {
        std::cerr << "ERROR: ExponentialDistributionEstimator: Invalid mean for the data:" << mean
                  << " Must be greater than 0.\n";
        return ExponentialDistribution(1.0, max_);
    }
}


TruncatedGaussianDistributionEstimator::TruncatedGaussianDistributionEstimator(double lower, double upper)
: lower_(lower)
, upper_(upper)
{
}


DistPtr
  TruncatedGaussianDistributionEstimator::estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                               const std::vector<double>::const_iterator dataEnd) const
{
    return DistPtr{new TruncatedGaussianDistribution(estimate(dataBegin, dataEnd))};
}


TruncatedGaussianDistribution
  TruncatedGaussianDistributionEstimator::estimate(const std::vector<double>::const_iterator dataBegin,
                                                   const std::vector<double>::const_iterator dataEnd) const
{
    double var = variance(dataBegin, dataEnd);

    if (var == 0.0) {
        std::cerr
          << "WARNING: TruncatedGaussianDistributionEstimator: Variance in the data was 0. Setting variance to 1e-4.\n";
        var = 1e-4;
    }

    return TruncatedGaussianDistribution(mean(dataBegin, dataEnd), var, lower_, upper_);
}

}   // namespace math
}   // namespace vulcan
