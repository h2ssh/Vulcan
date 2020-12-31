/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_distribution.cpp
* \author   Collin Johnson
*
* Definition of pose_distribution_t.
*/

#include "core/pose_distribution.h"
#include "core/float_comparison.h"

namespace vulcan
{

pose_distribution_t pose_distribution_t::compound(const pose_distribution_t& origin) const
{
    pose_distribution_t newDist;
    newDist.x = origin.x + (x * std::cos(origin.theta)) - (y * std::sin(origin.theta));
    newDist.y = origin.y + (x * std::sin(origin.theta)) + (y * std::cos(origin.theta));
    newDist.theta = angle_sum(origin.theta, theta);

    Matrix jThis = {
        { std::cos(origin.theta), -std::sin(origin.theta), 0.0 },
        { std::sin(origin.theta), std::cos(origin.theta), 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    Matrix jOrigin = {
        { 1.0, 0.0, (-x * std::sin(origin.theta)) - (y * std::cos(origin.theta)) },
        { 0.0, 1.0, (x * std::cos(origin.theta)) - (y * std::sin(origin.theta)) },
        { 0.0, 0.0, 1.0 }
    };

    Vector mean = { newDist.x, newDist.y, newDist. theta };
    Matrix cov(3, 3);
    cov.zeros();

    cov += jOrigin * origin.uncertainty.getCovariance() * arma::trans(jOrigin);
    cov += jThis * uncertainty.getCovariance() * arma::trans(jThis);
    newDist.uncertainty.setDistributionStatistics(mean, cov);

    return newDist;
}


bool operator==(const pose_distribution_t& lhs, const pose_distribution_t& rhs)
{
    return absolute_fuzzy_equal(lhs.x, rhs.x) &&
           absolute_fuzzy_equal(lhs.y, rhs.y) &&
           absolute_fuzzy_equal(lhs.theta, rhs.theta);
}


bool operator!=(const pose_distribution_t& lhs, const pose_distribution_t& rhs)
{
    return !(lhs == rhs);
}

} // namespace vulcan
