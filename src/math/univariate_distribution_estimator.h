/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     univariate_distribution_estimator.h
* \author   Collin Johnson
* 
* Definition of UnivariateDistributionEstimator interface.
*/

#ifndef MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_H
#define MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_H

#include <memory>
#include <vector>

namespace vulcan
{
namespace math
{
    
class UnivariateDistribution;

/**
* UnivariateDistributionEstimator is an interface for classes that estimate the parameters of a univariate distribution from
* a collection of samples.
* 
* The estimator has a single method that estimates a UnivariateDistribution from the provided data.
*/
class UnivariateDistributionEstimator
{
public:
    
    virtual ~UnivariateDistributionEstimator(void) { }
    
    /**
    * estimateDistribution estimates a new UnivariateDistribution from a collection of data.
    * 
    * \param    dataBegin           Start iterator of the data
    * \param    dataEnd             Beyond-the-end iterator for the data
    * \return   An instance of UnivariateDistribution estimated from the provided data.
    */
    virtual std::unique_ptr<UnivariateDistribution> estimateDistribution(const std::vector<double>::const_iterator dataBegin,
                                                                         const std::vector<double>::const_iterator dataEnd) const = 0;
};

}
}

#endif // MATH_UNIVARIATE_DISTRIBUTION_ESTIMATOR_H
