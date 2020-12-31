/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_MAHALANOBIS_H
#define MATH_MAHALANOBIS_H

#include "core/vector.h"

namespace vulcan
{
class MultivariateGaussian;

namespace math
{

/**
 * mahalanobis_distance calculates the Mahalanobis distance of a Vector from the provided Gaussian in terms
 * of the covariance of the Gaussian.
 */
float mahalanobis_distance(const MultivariateGaussian& gaussian, const Vector& vector);

/**
 * mahalanobis_distance calculates the Mahalanobis distance of a value from another value in terms of the
 * provided variance.
 */
float mahalanobis_distance(float mean, float variance, float value);

}   // namespace math
}   // namespace vulcan

#endif   // MATH_MAHALANOBIS_H
