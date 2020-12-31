/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exponential_distribution.cpp
* \author   Collin Johnson
* 
* Definition of ExponentialDistribution.
*/

#include "math/exponential_distribution.h"
#include <cmath>
#include <iostream>
#include <limits>

namespace vulcan
{
namespace math
{
    
double max_value(double value)                   { return value <= 0.0 ? std::numeric_limits<double>::infinity() : value; }
double etta     (double lambda, double maxValue) { return 1.0 / (1.0 - std::exp(-lambda * maxValue)); }


ExponentialDistribution::ExponentialDistribution(double lambda, double maxValue)
: lambda_(lambda)
{
    setMaxValue(maxValue);
}


void ExponentialDistribution::setMaxValue(double max)
{
    maxValue_ = max_value(max);
    etta_     = etta(lambda_, maxValue_);
}


double ExponentialDistribution::sample(void) const
{
    std::cout << "STUB: ExponentialDistribution::sample(void)\n";
    return 0.0;
}


double ExponentialDistribution::likelihood(double value) const
{
    if(value < 0.0 || value > maxValue_)
    {
        return 0.0;
    }
    
    return etta_ * lambda_ * std::exp(-lambda_ * value);
}


bool ExponentialDistribution::save(std::ostream& out) const
{
    out << lambda_ << ' ' << maxValue_ << '\n';
    return out.good();
}


bool ExponentialDistribution::load(std::istream& in)
{
    in >> lambda_ >> maxValue_;
    setMaxValue(maxValue_);
    return in.good();
}

}
}