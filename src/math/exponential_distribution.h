/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exponential_distribution.h
* \author   Collin Johnson
* 
* Declaration of ExponentialDistribution.
*/

#ifndef MATH_EXPONENTIAL_DISTRIBUTION_H
#define MATH_EXPONENTIAL_DISTRIBUTION_H

#include "math/univariate_distribution.h"

namespace vulcan
{
namespace math
{
    
const std::string kExponentialType{"exponential"};
    
/**
* ExponentialDistribution is an implementation of the exponential distribution:
* 
*   p(z) = etta * lambda * e^(-lambda*z)
* 
* This implementation supports a maximum value for z, which then sets etta appropriately.
* In the case where the range of z = [0, max),
* 
*   etta = 1 / (1 - e^(-lambda*max))
* 
* if the range of z = [0, inf), then etta = 1.
*/
class ExponentialDistribution : public UnivariateDistribution
{
public:
    
    /**
    * Constructor for ExponentialDistribution.
    * 
    * \param    lambda          Parameter controlling the distribution
    * \param    maxValue        Maximum value the distribution can take (optional)
    */
    explicit ExponentialDistribution(double lambda = 1.0, double maxValue = -1.0);
    
    /**
    * setMaxValue sets the maximum value for the distribution. If the max value is less than 0, it is assumed to be infinite.
    */
    void setMaxValue(double max);
    
    // UnivariateDistribution interface
    virtual std::string name      (void)              const override { return kExponentialType; }
    virtual double      sample    (void)              const override;
    virtual double      likelihood(double value)      const override;
    virtual bool        save      (std::ostream& out) const override;
    virtual bool        load      (std::istream& in)        override;
    
private:
    
    double lambda_;
    double maxValue_;
    double etta_;
};
    
}
}

#endif // MATH_EXPONENTIAL_DISTRIBUTION_H
