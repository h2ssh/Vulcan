/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     univariate_gaussian.h
* \author   Collin Johnson
*
* Declaration of UnivariateGaussianDistribution.
*/

#ifndef MATH_UNIVARIATE_GAUSSIAN_H
#define MATH_UNIVARIATE_GAUSSIAN_H

#include <math/univariate_distribution.h>
#include <math/statistics.h>
#include <random>

namespace vulcan
{
namespace math
{

const std::string kUnivariateGaussianType{"univariate_gaussian"};

/**
* UnivariateGaussian is a single-variable Gaussian, or normal distribution, described by two parameters:
*
*   mean (mu) = center of the distribution
*   variance (sigma^2) = variance of the distribution
*/
class UnivariateGaussianDistribution : public UnivariateDistribution
{
public:

    explicit UnivariateGaussianDistribution(double mean = 1.0, double variance = 1.0);

    template <typename ConstIter>
    UnivariateGaussianDistribution(ConstIter begin, ConstIter end)
    {
        mean_ = math::mean(begin, end);
        variance_ = math::variance(begin, end, mean_);
        normalizer_ = (variance_ > 0.0) ? univariateNormalizer(variance_) : HUGE_VAL;
    }

    void setMean    (double mean);
    void setVariance(double variance);

    double mean(void)     const { return mean_;     }
    double variance(void) const { return variance_; }

    // UnivariateDistribution interface
    virtual std::string name      (void)              const override { return kUnivariateGaussianType; }
    virtual double      sample    (void)              const override;
    virtual double      likelihood(double value)      const override;
    virtual bool        save      (std::ostream& out) const override;
    virtual bool        load      (std::istream& in)        override;

private:

    double mean_;
    double variance_;
    double normalizer_;

    mutable std::default_random_engine       generator_;
    mutable std::normal_distribution<double> distribution_;

    double univariateNormalizer(double variance) { return 1.0 / std::sqrt(2.0 * M_PI * variance); }
};

} // namespace math
} // namespace vulcan

#endif // MATH_UNIVARIATE_GAUSSIAN_H
