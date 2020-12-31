/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pose_distribution.h
 * \author   Collin Johnson
 *
 * Declaration of pose_distrubtion_t.
 */

#ifndef CORE_POSE_DISTRIBUTION_H
#define CORE_POSE_DISTRIBUTION_H

#include "core/multivariate_gaussian.h"
#include "core/pose.h"
#include "system/message_traits.h"

namespace vulcan
{

/**
 * pose_distribution_t represents the robot pose and uncertainty in Cartesian coordinates in the current LPM reference
 * frame. The pose is also stored outside the Gaussian for ease of access.
 */
struct pose_distribution_t
{
    int64_t timestamp;

    float x;
    float y;
    float theta;

    MultivariateGaussian uncertainty;

    pose_distribution_t(void) : timestamp(0), x(0), y(0), theta(0), uncertainty(3)
    {
        Vector pose(3);
        pose.zeros();
        Matrix cov(3, 3);
        cov.eye();
        cov *= 0.0000001;
        uncertainty.setDistributionStatistics(pose, cov);
    }

    pose_distribution_t(int64_t timestamp, float x, float y, float theta, const MultivariateGaussian& uncertainty)
    : timestamp(timestamp)
    , x(x)
    , y(y)
    , theta(theta)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);
    }

    pose_distribution_t(float x, float y, float theta, float varX, float varY, float varTheta)
    : timestamp(0)
    , x(x)
    , y(y)
    , theta(theta)
    , uncertainty(3)
    {
        assert(uncertainty.dimensions() == 3);
        Vector pose = {x, y, theta};
        Matrix cov = {{varX, 0.0, 0.0}, {0.0, varY, 0.0}, {0.0, 0.0, varTheta}};

        uncertainty.setDistributionStatistics(pose, cov);
    }

    pose_distribution_t(const pose_t& pose, const MultivariateGaussian& uncertainty)
    : timestamp(pose.timestamp)
    , x(pose.x)
    , y(pose.y)
    , theta(pose.theta)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);
    }

    pose_distribution_t(const MultivariateGaussian& uncertainty) : timestamp(0), uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);

        x = uncertainty[0];
        y = uncertainty[1];
        theta = uncertainty[2];
    }

    pose_distribution_t(int64_t timestamp, const MultivariateGaussian& uncertainty)
    : timestamp(timestamp)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);

        Vector pose = uncertainty.getMean();
        x = pose(0);
        y = pose(1);
        theta = pose(2);
    }

    // Create with default noise
    explicit pose_distribution_t(const pose_t& pose)
    : timestamp(pose.timestamp)
    , x(pose.x)
    , y(pose.y)
    , theta(pose.theta)
    , uncertainty(3)
    {
        Vector p = {pose.x, pose.y, pose.theta};
        Matrix cov(3, 3);
        cov.eye();
        cov *= 0.0000001;
        uncertainty.setDistributionStatistics(p, cov);
    }

    /**
     * Apply the compound operator to transform this pose relative to another origin. The covariance is correctly
     * adjusted in the translation.
     */
    pose_distribution_t compound(const pose_distribution_t& origin) const;

    /**
     * toPose converts the pose_distribution_t to a simple pose, which is used in most places in the code.
     */
    pose_t toPose(void) const { return pose_t(timestamp, x, y, theta); }

    /**
     * Convert to just the position.
     */
    Point<float> toPoint(void) const { return Point<float>(x, y); }
};

bool operator==(const pose_distribution_t& lhs, const pose_distribution_t& rhs);
bool operator!=(const pose_distribution_t& lhs, const pose_distribution_t& rhs);

// Serialization support
template <class Archive>
void serialize(Archive& ar, pose_distribution_t& pose)
{
    ar& pose.timestamp;
    ar& pose.x;
    ar& pose.y;
    ar& pose.theta;
    ar& pose.uncertainty;
}

}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(pose_distribution_t, ("HSSH_LOCAL_POSE_DISTRIBUTION"))

#endif   // CORE_POSE_DISTRIBUTION_H
