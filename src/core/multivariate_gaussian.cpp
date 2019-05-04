/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     multivariate_gaussian.cpp
* \author   Collin Johnson
*
* Definition of MultivariateGaussian.
*/

#include <core/multivariate_gaussian.h>
#include <cassert>

namespace vulcan
{

MultivariateGaussian::MultivariateGaussian(size_t dimensions)
    : mean(dimensions)
    , covariance(dimensions, dimensions)
{
}


MultivariateGaussian::MultivariateGaussian(const Vector& mean, const Matrix& covariance)
    : mean(mean)
    , covariance(covariance)
{
    assert(covariance.is_finite());
    assert(mean.is_finite());
}


void MultivariateGaussian::setDistributionStatistics(const Vector& mean, const Matrix& covariance)
{
    this->mean       = mean;
    this->covariance = covariance;

    // If the cholesky has already been computed for the distribution, chances are it will be used
    // again, so do the calculation right away
    if(covarianceCholesky.n_rows > 0)
    {
        covarianceCholesky = chol(covariance);
        assert(covarianceCholesky.is_finite());
        assert(covarianceCholesky.n_elem > 0);
    }
}


Vector MultivariateGaussian::sample(void) const
{
    Vector sampleMean = covarianceCholesky * arma::randn<Vector>(mean.n_rows);
    return sampleMean + mean;
}


void MultivariateGaussian::prepareForSampling(void)
{
    covarianceCholesky = chol(covariance);
    assert(covarianceCholesky.is_finite());
    assert(covarianceCholesky.n_elem > 0);
}


double MultivariateGaussian::probability(const Vector& vector) const
{
//     double coeff = 1.0 / sqrt(2 * M_PI * arma::det(covariance));
    Vector error = vector - mean;
//     return coeff * exp(-0.5*arma::as_scalar((arma::trans(error) * arma::inv(covariance) * error)));
    Matrix inv = arma::inv(covariance);
    return exp(-0.5*arma::as_scalar((arma::trans(error) * inv * error)));
}


double MultivariateGaussian::logPosterior(const Vector& vector) const
{
    Vector error = vector - mean;
    return -0.5*arma::as_scalar((arma::trans(error) * arma::inv(covariance) * error));
}

} // namespace vulcan
