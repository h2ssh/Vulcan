/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     beta_distribution.h
* \author   Collin Johnson
* 
* Declaration of BetaDistribution.
*/

#ifndef MATH_BETA_DISTRIBUTION_H
#define MATH_BETA_DISTRIBUTION_H

#include "math/univariate_distribution.h"

namespace vulcan
{
namespace math
{
    
const std::string kBetaType{"beta"};
    
/**
* BetaDistribution is a distribution defined in the interval [0, 1].
* 
*   B(x) = x^(alpha-1) * (1-x)^(beta-1) / Beta(alpha, beta)
* 
* where alpha and beta are parameters of the distribution and Beta is the beta function,
* which you can find readily described online.
*/
class BetaDistribution : public UnivariateDistribution
{
public:
    
    /**
    * Constructor for BetaDistribution.
    * 
    * \param    alpha       Alpha parameter for the distribution
    * \param    beta        Beta parameter for the distribution
    */
    explicit BetaDistribution(double alpha = 1.0, double beta = 1.0);
    
    // UnivariateDistribution interface
    virtual std::string name      (void)              const override { return kBetaType; }
    virtual double      sample    (void)              const override;
    virtual double      likelihood(double value)      const override;
    virtual bool        save      (std::ostream& out) const override;
    virtual bool        load      (std::istream& in)        override;
    
private:
    
    double alpha_;
    double beta_;
    double normalizer_;
};
    
}
}

#endif // MATH_BETA_DISTRIBUTION_H
