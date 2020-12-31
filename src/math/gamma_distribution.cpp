/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gamma_distribution.cpp
* \author   Collin Johnson
* 
* Implementation of GammaDistribution.
*/

#include "math/gamma_distribution.h"
#include "math/roots.h"
#include <boost/math/special_functions/gamma.hpp>
#include <algorithm>
#include <functional>
#include <iostream>

namespace vulcan
{
namespace math
{

double gamma_normalizer(double k, double theta) { return 1.0 / (boost::math::tgamma(k) * std::pow(theta, k)); }


GammaDistribution::GammaDistribution(double k, double theta)
: k_(k)
, theta_(theta)
{
    assert(k_ > 0.0);
    assert(theta_ > 0.0);
    
    normalizer_ = gamma_normalizer(k_, theta_);
}


double GammaDistribution::sample(void) const
{
    std::cout << "STUB: GammaDistribution::sample(void)\n";
    return 0.0;
}


double GammaDistribution::likelihood(double value) const
{
    return normalizer_ * std::pow(value, k_-1.0) * std::exp(-value / theta_);
}


bool GammaDistribution::save(std::ostream& out) const
{
    out << k_ << ' ' << theta_ << '\n';
    return out.good();
}


bool GammaDistribution::load(std::istream& in)
{
    in >> k_ >> theta_;
    
    assert(k_ > 0.0);
    assert(theta_ > 0.0);
    
    normalizer_ = gamma_normalizer(k_, theta_);
    
    return in.good();
}

}
}
