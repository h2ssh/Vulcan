/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     truncated_gaussian_distribution.cpp
* \author   Collin Johnson
* 
* Definition of TruncatedGaussianDistribution.
*/

#include "math/truncated_gaussian_distribution.h"
#include "math/statistics.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace vulcan
{
namespace math
{
    
double gaussian_cdf        (double x, double mean, double sigma);
double truncated_normalizer(double a, double b, double mean, double variance);
    
    
TruncatedGaussianDistribution::TruncatedGaussianDistribution(double mean, double variance, double a, double b)
: mean_(mean)
, variance_(variance)
, a_(a)
, b_(b)
{
    assert(a_ < b_);
    assert(variance_ > 0.0);
    
    normalizer_ = truncated_normalizer(a_, b_, mean_, variance_);
    sigma_ = std::sqrt(variance);
    aCdf_ = gaussian_cdf(a_, mean_, sigma_);
    cdfNormalizer_ = gaussian_cdf(b_, mean_, sigma_) - aCdf_;
}


double TruncatedGaussianDistribution::cdf(double x) const
{
    return cdfNormalizer_ * (gaussian_cdf(x, mean_, sigma_) - aCdf_);
}


double TruncatedGaussianDistribution::sample(void) const
{
    std::cout << "STUB: TruncatedGaussianDistribution::sample(void)\n";
    return 0.0;
}


double TruncatedGaussianDistribution::likelihood(double value) const
{
    if((value < a_) || (value > b_))
    {
        return 0.0;
    }
    
    return normalizer_ * std::exp(-0.5 * std::pow(value - mean_, 2.0) / variance_);
}


bool TruncatedGaussianDistribution::save(std::ostream& out) const
{
    out << mean_ << ' ' << variance_ << ' ' << a_ << ' ' << b_ << '\n';
    return out.good();
}


bool TruncatedGaussianDistribution::load(std::istream& in)
{
    in >> mean_ >> variance_ >> a_ >> b_;
    
    assert(a_ < b_);
    assert(variance_ > 0.0);
    
    normalizer_ = truncated_normalizer(a_, b_, mean_, sigma_);
    
    return in.good();
}


double gaussian_cdf(double x, double mean, double sigma)
{ 
    return 0.5 * (1.0 + std::erf((x - mean) / (M_SQRT2 * sigma))); 
}


double truncated_normalizer(double a, double b, double mean, double sigma)
{
    return 1.0 / (sigma * (gaussian_cdf(b, mean, sigma) - gaussian_cdf(a, mean, sigma)) * std::sqrt(2.0 * M_PI) * sigma);
}
    
} // namespace math
} // namespace vulcan