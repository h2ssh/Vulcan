/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     beta_distribution.cpp
* \author   Collin Johnson
* 
* Definition of BetaDistribution.
*/

#include <math/beta_distribution.h>
#include <boost/math/special_functions/beta.hpp>
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace math
{
    
double beta_normalizer(double alpha, double beta) { return 1.0 / boost::math::beta(alpha, beta); }


BetaDistribution::BetaDistribution(double alpha, double beta)
: alpha_(alpha)
, beta_(beta)
, normalizer_(beta_normalizer(alpha_, beta_))
{
    assert(alpha_ >= 0.0);
    assert(beta_  >= 0.0);
}


double BetaDistribution::sample(void) const
{
    std::cout << "STUB: BetaDistribution::sample(void)\n";
    return 0.0;
}


double BetaDistribution::likelihood(double value) const
{
    if((value <= 0.0) || (value >= 1.0))
    {
        return 0.0;
    }
    
    return std::pow(value, alpha_-1.0) * std::pow(1.0-value, beta_-1.0) * normalizer_;
}


bool BetaDistribution::save(std::ostream& out) const
{
    out << alpha_ << ' ' << beta_ << '\n';
    return out.good();
}


bool BetaDistribution::load(std::istream& in)
{
    in >> alpha_ >> beta_;
    
    assert(alpha_ >= 0.0);
    assert(beta_  >= 0.0);
    
    normalizer_ = beta_normalizer(alpha_, beta_);
    
    return in.good();
}

}
}
