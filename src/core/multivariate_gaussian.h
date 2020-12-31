/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     multivariate_gaussian.h
 * \author   Collin Johnson
 *
 * Declaration of a simple MultivariateGaussian implementation.
 */

#ifndef CORE_MULTIVARIATE_GAUSSIAN_H
#define CORE_MULTIVARIATE_GAUSSIAN_H

#include "core/matrix.h"
#include "core/vector.h"
#include <cereal/access.hpp>

namespace vulcan
{

/**
 * MultivariateGaussian is a multi-dimensional Gaussian distribution described by a mean and covariance.
 *
 * Samples can be drawn from the distribution using the sample() method.
 */
class MultivariateGaussian
{
public:
    /**
     * Default constructor for MultivariateGaussian.
     *
     * \param    dimensions      Number of dimensions for the Gaussian (default = 2)
     */
    explicit MultivariateGaussian(std::size_t dimensions = 2);

    /**
     * Constructor for MultivariateGaussian.
     */
    MultivariateGaussian(const Vector& mean, const Matrix& covariance);

    /**
     * setDistributionStatistics sets statistics for the distribution. Both the mean
     * and covariance are required because it isn't a distribution otherwise!
     */
    void setDistributionStatistics(const Vector& mean, const Matrix& covariance);

    // Accessors

    Vector getMean(void) const { return mean; }
    Matrix getCovariance(void) const { return covariance; }
    std::size_t dimensions(void) const { return mean.n_rows; }

    /**
     * sample draws a sample from the distribution.
     */
    Vector sample(void) const;

    /**
     * prepareForSampling needs to be called before the first call to sample().
     */
    void prepareForSampling(void);

    /**
     * probability evaluates the probability of the PDF at the specified value.
     */
    double probability(const Vector& value) const;

    /**
     * logPosterior evaluates the natural log of probability of the PDF at the specified value.
     */
    double logPosterior(const Vector& value) const;

    // Operator overloads:

    /**
     * operator[] provides access to the corresponding element of the mean vector for the Gaussian.
     *
     * \pre  0 <= n < dimensions()
     */
    double operator[](int n) const { return mean(n); }
    double& operator[](int n) { return mean(n); }

    /**
     * operator() provides access to the corresponding element of the covariance matrix for the Gaussian.
     *
     * \pre  0 <= row < dimensions()
     * \pre  0 <= col < dimensions()
     */
    double operator()(int row, int col) const { return covariance(row, col); }
    double& operator()(int row, int col) { return covariance(row, col); }

private:
    Vector mean;
    Matrix covariance;

    Matrix covarianceCholesky;   // Cholesky factorization of covariance -- used for sampling

    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& mean;
        ar& covariance;
    }
};

}   // namespace vulcan

#endif   // CORE_MULTIVARIATE_GAUSSIAN_H
