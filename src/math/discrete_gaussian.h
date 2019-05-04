/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discrete_gaussian.h
* \author   Collin Johnson
* 
* Declaration of DiscreteGaussianDistribution.
*/

#ifndef MATH_DISCRETE_GAUSSIAN_H
#define MATH_DISCRETE_GAUSSIAN_H

#include <math/univariate_distribution.h>

namespace vulcan
{
namespace math
{
    
const std::string kDiscreteGaussianType{"discrete_gaussian"};

/**
* DiscreteGaussianDistribution uses a continuous Gaussian distribution to caluclate a discrete normal distribution
* for discrete values. The mean and variance are calculated as normal. The likelihood term is calculated by treating
* the discrete value as the center of a bin of probability and intergrating the pdf through the range.
* 
* See http://www.milefoot.com/math/stat/pdfc-normaldisc.htm   for details.
*/
class DiscreteGaussianDistribution : public UnivariateDistribution
{
public:
    
    /**
    * Constructor for DiscreteGaussianDistribution.
    * 
    * \param    mean            Mean of the distribution
    * \param    variance        Variance of the distribution
    * 
    * \pre variance > 0
    */
    explicit DiscreteGaussianDistribution(double mean = 0.0, double variance = 0.0);
    
    // UnivariateDistribution interface
    virtual std::string name      (void)              const override { return kDiscreteGaussianType; }
    virtual double      sample    (void)              const override;
    virtual double      likelihood(double value)      const override;
    virtual bool        save      (std::ostream& out) const override;
    virtual bool        load      (std::istream& in)        override;

private:

    double mean_;
    double variance_;
};

}
}

#endif // MATH_DISCRETE_GAUSSIAN_H
