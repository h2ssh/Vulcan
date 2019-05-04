/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discrete_gaussian.cpp
* \author   Collin Johnson
* 
* Definition of DiscreteGaussianDistribution.
*/

#include <math/discrete_gaussian.h>
#include <cmath>
#include <iostream>

namespace vulcan
{
namespace math
{
    
double cdf(double x, double mean, double variance);
    

DiscreteGaussianDistribution::DiscreteGaussianDistribution(double mean, double variance)
: mean_(mean)
, variance_(variance)
{
}
    
    
double DiscreteGaussianDistribution::sample(void) const
{
    std::cerr << "STUB: DiscreteGaussianDistribution::sample(void)\n";
    return 0.0;
}


double DiscreteGaussianDistribution::likelihood(double value) const
{
    double upper = cdf(value + 0.5, mean_, variance_);
    double lower = cdf(value - 0.5, mean_, variance_);
    
    return upper - lower;
}


bool DiscreteGaussianDistribution::save(std::ostream& out) const
{
    out << mean_ << ' ' << variance_ << '\n';
    return out.good();
}


bool DiscreteGaussianDistribution::load(std::istream& in)
{
    in >> mean_ >> variance_;
    return in.good();
}


double cdf(double x, double mean, double variance)
{ 
    return 0.5 * (1.0 + std::erf((x - mean) / std::sqrt(2.0*variance))); 
    
}

}
}
