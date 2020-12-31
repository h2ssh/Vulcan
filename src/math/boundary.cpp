/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary.cpp
* \author   Collin Johnson
*
* Definition of functions for boundary probability distributions:
*
*/

#include "math/boundary.h"
#include "math/truncated_gaussian_distribution.h"
#include "core/angle_functions.h"
#include "core/vector.h"
#include <cassert>
#include <cmath>

namespace vulcan
{
namespace math
{

double heading_uncertainty(double vx, double vy, const Matrix& sigma)
{
    assert(std::abs(vx) + std::abs(vy) > 0.0);

    double denom = (vx * vx) + (vy * vy);
    Vector jacobian{-vy / denom, vx / denom};

    Matrix result = arma::trans(jacobian) * sigma * jacobian;
    return result(0, 0);
}


angle_range_t boundary_heading_range(const Line<double>& boundary,
                                     const Point<double>& position,
                                     double heading)
{
    // The range goes from the first boundary endpoint to the second. Because it is a line segment, the extent can be
    // at most pi radians, so there's no worry about accidentally creating the wrong angle range.
    angle_range_t range(angle_diff(angle_to_point(position, boundary.a), heading));
    range.expand(angle_diff(angle_to_point(position, boundary.b), heading));
    return range;
}


double boundary_heading_probability(const angle_range_t& range, double sigma)
{
    assert(range.extent <= M_PI);
    assert(sigma > 0.0);

//     std::cout << "Boundary probability: Range: " << range.start << " to " << (range.start + range.extent) << ": ";

    TruncatedGaussianDistribution dist(0.0, sigma*sigma, -M_PI, M_PI);

    // Wraparound will occur if start + extent > pi
    if(range.start + range.extent > M_PI)
    {
        auto wrappedEnd = wrap_to_pi(range.start + range.extent);
        // Computation is: (dist.cdf(M_PI) - dist.cdf(range.start)) + (dist.cdf(wrappedEnd) - dist.cdf(-M_PI));
        // but for truncated gaussian, dist.cdf(-M_PI) == 0.0, dist.cdf(M_PI) == 1.0

//         std::cout << "wrapped: start:" << dist.cdf(range.start) << " end:" << dist.cdf(wrappedEnd) << '\n';

        return 1.0 - dist.cdf(range.start) + dist.cdf(wrappedEnd);
    }
    // Otherwise can use simple CDF function and call it good
    else
    {
//         std::cout << "start: " << dist.cdf(range.start) << " end:" << dist.cdf(range.start + range.extent) << '\n';

        return dist.cdf(range.start + range.extent) - dist.cdf(range.start);
    }
}

} // namespace math
} // namespace vulcan
