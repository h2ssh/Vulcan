/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lambda.cpp
 * \author   Collin Johnson
 *
 * Definition of various functions for calculating the lambda value between two places.
 */

#include "hssh/local_topological/lambda.h"
#include "core/angle_functions.h"
#include "core/pose.h"
#include "core/pose_distribution.h"
#include <cmath>

#define DEBUG_MERGE

#ifdef DEBUG_MERGE
    #include <iostream>
#endif

const double kFixedNoiseRatio = 0.05;
const double kFixedPosNoise = 0.5;
const double kFixedThetaNoise = 0.1;

namespace vulcan
{
namespace hssh
{

Lambda::Lambda(float x, float y, float theta, float xVariance, float yVariance, float thetaVariance)
: x(x)
, y(y)
, theta(theta)
, xVariance(xVariance)
, yVariance(yVariance)
, thetaVariance(thetaVariance)
, transformDistribution(3)
{
    setupDistribution();
}


Lambda::Lambda(const pose_t& end, const pose_t& reference) : transformDistribution(3)
{
    auto endInRef = end.transformToNewFrame(reference);
    x = endInRef.x;
    y = endInRef.y;
    theta = endInRef.theta;

    setupDistribution();
}


Lambda::Lambda(const pose_distribution_t& end, const pose_distribution_t& reference)
: Lambda(end.toPose(), reference.toPose())
{
    Matrix cov = reference.uncertainty.getCovariance();
    cov += end.uncertainty.getCovariance();
    auto mean = transformDistribution.getMean();
    transformDistribution.setDistributionStatistics(mean, cov);
}


pose_distribution_t Lambda::apply(const pose_distribution_t& origin) const
{
    pose_distribution_t relative(x, y, theta, xVariance, yVariance, thetaVariance);
    return relative.compound(origin);
}


Lambda Lambda::invert(void) const
{
    Lambda inverted;

    const double cosAngle = cos(-theta);
    const double sinAngle = sin(-theta);

    inverted.theta = -theta;
    inverted.x = -x * cosAngle + y * sinAngle;
    inverted.y = -x * sinAngle - y * cosAngle;

    Vector invertedMean = transformDistribution.getMean();
    invertedMean(0) = inverted.x;
    invertedMean(1) = inverted.y;
    invertedMean(2) = inverted.theta;

    Matrix jacobian(3, 3);
    jacobian(0, 0) = -cosAngle;
    jacobian(0, 1) = -sinAngle;
    jacobian(0, 2) = -x * sinAngle + y * cosAngle;
    jacobian(1, 0) = -sinAngle;
    jacobian(1, 1) = -cosAngle;
    jacobian(1, 2) = -x * cosAngle + y * sinAngle;
    jacobian(2, 0) = 0;
    jacobian(2, 1) = 0;
    jacobian(2, 2) = 1;

    Matrix invertedCov(transformDistribution.getCovariance());
    invertedCov = arma::trans(jacobian) * invertedCov * jacobian;

    inverted.transformDistribution.setDistributionStatistics(invertedMean, invertedCov);

    return inverted;
}


Lambda Lambda::rotate(float angle) const
{
    Lambda rotated;

    const double cosAngle = cos(angle);
    const double sinAngle = sin(angle);

    rotated.x = x * cosAngle - y * sinAngle;
    rotated.y = x * sinAngle + y * cosAngle;
    rotated.theta = angle_sum(rotated.theta, angle);

    // Now need to rotate the covariance matrix. Take the Jacobian of the applied transform to get
    // the covariance in the new frame of reference
    Vector rotatedMean = transformDistribution.getMean();
    rotatedMean(0) = rotated.x;
    rotatedMean(1) = rotated.y;
    rotatedMean(2) = rotated.theta;

    Matrix jacobian(3, 3);
    jacobian(0, 0) = cosAngle;
    jacobian(0, 1) = -sinAngle;
    jacobian(0, 2) = -x * sinAngle - y * cosAngle;
    jacobian(1, 0) = sinAngle;
    jacobian(1, 1) = cosAngle;
    jacobian(1, 2) = x * cosAngle - y * sinAngle;
    jacobian(2, 0) = 0;
    jacobian(2, 1) = 0;
    jacobian(2, 2) = 1;

    Matrix rotatedCov(transformDistribution.getCovariance());
    rotatedCov = arma::trans(jacobian) * rotatedCov * jacobian;

    rotated.transformDistribution.setDistributionStatistics(rotatedMean, rotatedCov);

    rotated.xVariance = rotatedCov(0, 0);
    rotated.yVariance = rotatedCov(1, 1);
    rotated.thetaVariance = rotatedCov(2, 2);

    return rotated;
}


double Lambda::magnitude(void) const
{
    return std::sqrt(x * x + y * y);
}


double Lambda::heading(void) const
{
    return std::atan2(y, x);
}


void Lambda::merge(const Lambda& newLambda)
{
    Vector origMean = transformDistribution.getMean();
    Vector newMean = newLambda.transformDistribution.getMean();
    Matrix origCov = transformDistribution.getCovariance();
    Matrix newCov = newLambda.transformDistribution.getCovariance();

    newCov = origCov + newCov;

    newMean(2) = wrap_to_pi(newMean(2));
    transformDistribution.setDistributionStatistics(newMean, newCov);

#ifdef DEBUG_MERGE
    std::cout << "DEBUG:Lambda::merge: Orig:" << *this << '\n';
#endif

    x = newMean(0);
    y = newMean(1);
    theta = newMean(2);

    xVariance = newCov(0, 0);
    yVariance = newCov(1, 1);
    thetaVariance = newCov(2, 2);

#ifdef DEBUG_MERGE
    std::cout << " Merged:" << *this;
#endif
}


void Lambda::setupDistribution(void)
{
    xVariance = (x * kFixedNoiseRatio) + kFixedPosNoise;
    yVariance = (y * kFixedNoiseRatio) + kFixedPosNoise;
    thetaVariance = (theta * kFixedNoiseRatio) + kFixedThetaNoise;

    Vector transform(3);
    transform(0) = x;
    transform(1) = y;
    transform(2) = theta;

    Matrix covariance(3, 3);
    covariance.zeros();
    covariance(0, 0) = xVariance;
    covariance(1, 1) = yVariance;
    covariance(2, 2) = thetaVariance;

    transformDistribution.setDistributionStatistics(transform, covariance);
}

}   // namespace hssh
}   // namespace vulcan
