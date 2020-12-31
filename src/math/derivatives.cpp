/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     derivatives.h
 * \author   Collin Johnson
 *
 * Definition of utility function for calculating various derivatives:
 *
 *   - uncertain_time_derivative : take a time derivative between two uncertain measurements
 */

#include "math/derivatives.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace math
{

MultivariateGaussian
  uncertain_time_derivative(const MultivariateGaussian& start, const MultivariateGaussian& end, int64_t durationUs)
{
    assert(durationUs > 0);

    double deltaTime = utils::usec_to_sec(durationUs);

    Vector derivMean = (end.getMean() - start.getMean()) / deltaTime;
    Matrix derivCov = (end.getCovariance() + start.getCovariance()) / (deltaTime * deltaTime);

    return MultivariateGaussian(derivMean, derivCov);
}

}   // namespace math
}   // namespace vulcan
