/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     univariate_distribution_estimator_impl.h
* \author   Collin Johnson
* 
* Declaration of implementations of the UnivariateDistributionEstimator for the subclasses of UnivariateDistribution:
* 
*   - UnivariateGaussianDistribution
*   - GammaDistribution
*   - BetaDistribution
*   - BetaPrimeDistribution
*   - ExponentialDistribution
*   - TruncatedGaussianDistribution
*/

#ifndef MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_IMPL_H
#define MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_IMPL_H

#include "math/univariate_distribution_estimator.h"

namespace vulcan
{
namespace math
{
    
class UnivariateGaussianDistribution;
class DiscreteGaussianDistribution;
class GammaDistribution;
class BetaDistribution;
class ExponentialDistribution;
class TruncatedGaussianDistribution;

/**
* UnivariateGaussianDistributionEstimator
*/
class UnivariateGaussianDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    UnivariateGaussianDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                                            const std::vector<double>::const_iterator dataEnd) const;
};

/**
* DiscreteGaussianDistributionEstimator
*/
class DiscreteGaussianDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    DiscreteGaussianDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                                          const std::vector<double>::const_iterator dataEnd) const;
};


/**
* GammaDistributionEstimator
*/
class GammaDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    GammaDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                               const std::vector<double>::const_iterator dataEnd) const;
};

/**
* BetaDistributionEstimator
*/
class BetaDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    BetaDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                              const std::vector<double>::const_iterator dataEnd) const;
};

/**
* ExponentialDistributionEstimator
*/
class ExponentialDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    /**
    * Constructor for ExponentialDistributionEstimator.
    * 
    * \param    maxValue            Maximum value the exponential can take
    */
    ExponentialDistributionEstimator(double maxValue);
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    ExponentialDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                                     const std::vector<double>::const_iterator dataEnd) const;
                                     
private:
    
    double max_;
};

/**
* TruncatedGaussianDistributionEstimator
*/
class TruncatedGaussianDistributionEstimator : public UnivariateDistributionEstimator
{
public:
    
    /**
    * Constructor for TruncatedGaussianDistributionEstimator.
    * 
    * \param    lower       Lower bound of the support region
    * \param    upper       Upper bound of the support region
    */
    TruncatedGaussianDistributionEstimator(double lower, double upper);
    
    // UnivariateDistributionEstimator interface
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const override;
                                                                         
    TruncatedGaussianDistribution estimate(const std::vector<double>::const_iterator dataBegin,
                                           const std::vector<double>::const_iterator dataEnd) const;
                                           
private:
    
    double lower_;
    double upper_;
};

} // namespace math
} // namespace vulcan

#endif // MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_IMPL_H
