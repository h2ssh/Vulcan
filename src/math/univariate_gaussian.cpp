/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     univariate_gaussian.cpp
 * \author   Collin Johnson
 *
 * Definition of UnivariateGaussianDistribution.
 */

#include "math/univariate_gaussian.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace vulcan
{
namespace math
{

UnivariateGaussianDistribution::UnivariateGaussianDistribution(double mean, double variance)
: mean_(mean)
, variance_(variance)
, distribution_(mean, std::sqrt(variance))
{
    assert(variance_ > 0.0);
    normalizer_ = univariateNormalizer(variance_);
}


void UnivariateGaussianDistribution::setMean(double mean)
{
    mean_ = mean;
    distribution_ = std::normal_distribution<double>(mean_, std::sqrt(variance_));
}


void UnivariateGaussianDistribution::setVariance(double variance)
{
    variance_ = variance;
    distribution_ = std::normal_distribution<double>(mean_, std::sqrt(variance_));
}


double UnivariateGaussianDistribution::sample(void) const
{
    return distribution_(generator_);
}


double UnivariateGaussianDistribution::likelihood(double value) const
{
    return normalizer_ * std::exp(-std::pow(value - mean_, 2.0) / (2.0 * variance_));
}


bool UnivariateGaussianDistribution::save(std::ostream& out) const
{
    out << mean_ << ' ' << variance_ << '\n';
    return out.good();
}


bool UnivariateGaussianDistribution::load(std::istream& in)
{
    in >> mean_ >> variance_;
    assert(variance_ > 0.0);
    normalizer_ = univariateNormalizer(variance_);
    return in.good();
}

}   // namespace math
}   // namespace vulcan
