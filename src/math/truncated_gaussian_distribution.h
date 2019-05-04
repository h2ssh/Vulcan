/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     truncated_gaussian_distribution.h
* \author   Collin Johnson
* 
* Definition of TruncatedGaussianDistribution.
*/

#ifndef MATH_TRUNCATED_GAUSSIAN_DISTRIBUTION_H
#define MATH_TRUNCATED_GAUSSIAN_DISTRIBUTION_H

#include <math/univariate_distribution.h>

namespace vulcan
{
namespace math
{

const std::string kTruncatedGaussianType{"truncated_gaussian"};
    
/**
* TruncatedGaussianDistribution is an implementation of a truncated Gaussian distribution. Following the general theory
* of truncated distributions, the truncated Gaussian with domain [a, b] is defined as:
* 
*   T(x) =  (1/sigma) * n(x) / (N(b) - N(a))
* 
* where n(x) is the pdf univariate Gaussian with parameters, (mu, sigma), and N(x) is the CDF of the same univariate Gaussian.
* 
* Thus, T(x) contains four parametes: mu, sigma, a, b, all of which must be specified.
*/ 
class TruncatedGaussianDistribution : public UnivariateDistribution
{
public:
    
    /**
    * Constructor for TruncatedGaussianDistribution.
    * 
    * \param    mean        Mean of the distribution
    * \param    variance    Variance of the distribution (sqrt(variance))
    * \param    a           Lower bound of the domain of the support
    * \param    b           Upper bound of the domain of the support
    */
    explicit TruncatedGaussianDistribution(double mean = 0.5, double variance = 0.25, double a = 0.0, double b = 1.0);
    
    // Observers for the parameters
    double mean(void)     const { return mean_; }
    double variance(void) const { return variance_; }
    double a(void)        const { return a_; }
    double b(void)        const { return b_; }
    
    double cdf(double x) const;
    
    // UnivariateDistribution interface
    virtual std::string name      (void)              const override { return kTruncatedGaussianType; }
    virtual double      sample    (void)              const override;
    virtual double      likelihood(double value)      const override;
    virtual bool        save      (std::ostream& out) const override;
    virtual bool        load      (std::istream& in)        override;
    
private:
    
    double mean_;
    double variance_;
    double a_;
    double b_;
    double normalizer_;
    double cdfNormalizer_;
    double sigma_;
    double aCdf_;
};
    
}
}

#endif // MATH_TRUNCATED_GAUSSIAN_DISTRIBUTION_H
