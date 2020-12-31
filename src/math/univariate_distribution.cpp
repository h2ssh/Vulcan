/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     univariate_distribution.cpp
* \author   Collin Johnson
* 
* Definition of the create_univariate_distribution factory function.
*/

#include "math/univariate_distribution.h"
#include "math/discrete_gaussian.h"
#include "math/beta_distribution.h"
#include "math/gamma_distribution.h"
#include "math/univariate_gaussian.h"
#include "math/exponential_distribution.h"
#include "math/truncated_gaussian_distribution.h"
#include <iostream>

namespace vulcan
{
namespace math
{

std::unique_ptr<UnivariateDistribution> create_univariate_distribution(const std::string& type)
{
    using Ptr = std::unique_ptr<UnivariateDistribution>;

    if(type == kBetaType)
    {
        return Ptr{new BetaDistribution()};
    }
    else if(type == kExponentialType)
    {
        return Ptr{new ExponentialDistribution()};
    }
    else if(type == kTruncatedGaussianType)
    {
        return Ptr{new TruncatedGaussianDistribution()};
    }
    else if(type == kUnivariateGaussianType)
    {
        return Ptr{new UnivariateGaussianDistribution()};
    }
    else if(type == kGammaType)
    {
        return Ptr{new GammaDistribution()};
    }
    else if(type == kDiscreteGaussianType)
    {
        return Ptr{new DiscreteGaussianDistribution()};
    }
    else
    {
        std::cerr << "ERROR: create_univariate_distribution: Unknown distribution type: " << type << '\n';
    }

    return Ptr{};
}

}
}
