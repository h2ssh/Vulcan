/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gamma_distribution.h
 * \author   Collin Johnson
 *
 * Definition of GammaDistribution.
 */

#ifndef MATH_GAMMA_DISTRIBUTION_H
#define MATH_GAMMA_DISTRIBUTION_H

#include "math/univariate_distribution.h"

namespace vulcan
{
namespace math
{

const std::string kGammaType{"gamma"};

/**
 * GammaDistribution is a distribution defined in the interval (0, inf] and parameters {k, theta}
 *
 *   G(x) = (1/(gamma(k)*theta^k)) * x^(k-1) * e^(-x / theta)
 *
 * where gamma() is the gamma function and k > 0, theta > 0
 */
class GammaDistribution : public UnivariateDistribution
{
public:
    /**
     * Constructor for GammaDistribution.
     *
     * \param    k           k-parameter
     * \param    theta       theta-parameter
     */
    explicit GammaDistribution(double k = 1.0, double theta = 1.0);

    // Observers for the parameters
    double k(void) const { return k_; }
    double theta(void) const { return theta_; }

    // UnivariateDistribution interface
    virtual std::string name(void) const override { return kGammaType; }
    virtual double sample(void) const override;
    virtual double likelihood(double value) const override;
    virtual bool save(std::ostream& out) const override;
    virtual bool load(std::istream& in) override;

private:
    double k_;
    double theta_;
    double normalizer_;
};

}   // namespace math
}   // namespace vulcan

#endif   // MATH_GAMMA_DISTRIBUTION_H
