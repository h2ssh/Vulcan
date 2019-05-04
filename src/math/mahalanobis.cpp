/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cmath>
#include <core/multivariate_gaussian.h>
#include <math/mahalanobis.h>

namespace vulcan
{
namespace math
{

float mahalanobis_distance(const MultivariateGaussian& gaussian, const Vector& vector)
{
    Vector delta = vector - gaussian.getMean();
    Vector maha  = arma::trans(delta) * arma::inv(gaussian.getCovariance()) * delta;

    return sqrt(maha(0));
}


float mahalanobis_distance(float mean, float variance, float value)
{
    return sqrt(pow(value-mean, 2) / variance);
}

}
}
