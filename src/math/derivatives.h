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
* Declaration of utility function for calculating various derivatives:
*
*   - uncertain_time_derivative : take a time derivative between two uncertain measurements
*/

#ifndef MATH_DERIVATIVES_H
#define MATH_DERIVATIVES_H

#include <core/multivariate_gaussian.h>

namespace vulcan
{
namespace math
{

/**
* uncertain_time_derivative takes a time derivative of the provided uncertain values. The time derivative is taken in
* seconds.
*
* \param    start           Start value of the uncertain value
* \param    end             End value of the uncertain value
* \param    durationUs      Time elapsed between measurement of start and end
* \pre durationUs > 0
* \return   Uncertain derivative with respect to time measured in seconds.
*/
MultivariateGaussian uncertain_time_derivative(const MultivariateGaussian& start,
                                               const MultivariateGaussian& end,
                                               int64_t durationUs);

} // namespace math
} // namespace vulcan

#endif // MATH_DERIVATIVES_H
