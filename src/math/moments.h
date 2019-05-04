/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     moments.h
* \author   Collin Johnson
*
* Definition of functions for calculating the moments of collections of points.
*/

#ifndef MATH_MOMENTS_H
#define MATH_MOMENTS_H

#include <core/point.h>
#include <algorithm>
#include <array>
#include <stdexcept>

namespace vulcan
{
namespace math
{

enum MomentIndex
{
    kM11,
    kM20,
    kM02,
    kM12,
    kM21,
    kM30,
    kM03,
    kNumMoments
};

using SecondOrderMoments = std::array<double, 3>;
using ThirdOrderMoments  = std::array<double, kNumMoments>;

/**
 * moment
 *
 * \throws invalid_argument if area <= 0.0
 */
template <class PointIterator>
double central_moment(PointIterator begin, PointIterator end, int xExp, int yExp, const Point<double>& center)
{
    return std::accumulate(begin,
                           end,
                           0.0,
                           [=](double total, const Point<double>& point)
                           {
                               return total + std::pow(point.x-center.x, xExp)*std::pow(point.y-center.y, yExp);

                           });
}

/**
 * scale_invariant_moment
 */
template <class PointIterator>
double scale_invariant_moment(PointIterator begin, PointIterator end, int xExp, int yExp, double area, const Point<double>& center)
{
    if(area <= 0.0)
    {
        throw std::invalid_argument("moment: area must be greater than 0.");
    }

    return central_moment(begin, end, xExp, yExp, center) / std::pow(area, 1 + (xExp+yExp)/2);
}

/**
* second_order_central_moments
*
* \throws invalid_argument if area <= 0.0
*/
template <class PointIterator>
SecondOrderMoments second_order_central_moments(PointIterator begin,
                                                PointIterator end,
                                                const Point<double>& center)
{
    std::array<double, 3> moments;

    moments[kM11] = central_moment(begin, end, 1, 1, center);
    moments[kM20] = central_moment(begin, end, 2, 0, center);
    moments[kM02] = central_moment(begin, end, 0, 2, center);

    return moments;
}

/**
* third_order_central_moments
*
* \throws invalid_argument if area <= 0.0
*/
template <class PointIterator>
ThirdOrderMoments third_order_central_moments(PointIterator begin,
                                              PointIterator end,
                                              const Point<double>& center)
{
    ThirdOrderMoments moments;

    moments[kM11] = central_moment(begin, end, 1, 1, center);
    moments[kM20] = central_moment(begin, end, 2, 0, center);
    moments[kM02] = central_moment(begin, end, 0, 2, center);
    moments[kM12] = central_moment(begin, end, 1, 2, center);
    moments[kM21] = central_moment(begin, end, 2, 1, center);
    moments[kM30] = central_moment(begin, end, 3, 0, center);
    moments[kM03] = central_moment(begin, end, 0, 3, center);

    return moments;
}

/**
* third_order_scale_invariant_moments
*/
template <class PointIterator>
ThirdOrderMoments third_order_scale_invariant_moments(PointIterator begin,
                                                      PointIterator end,
                                                      double area,
                                                      const Point<double>& center)
{
    ThirdOrderMoments moments;

    moments[kM11] = scale_invariant_moment(begin, end, 1, 1, area, center);
    moments[kM20] = scale_invariant_moment(begin, end, 2, 0, area, center);
    moments[kM02] = scale_invariant_moment(begin, end, 0, 2, area, center);
    moments[kM12] = scale_invariant_moment(begin, end, 1, 2, area, center);
    moments[kM21] = scale_invariant_moment(begin, end, 2, 1, area, center);
    moments[kM30] = scale_invariant_moment(begin, end, 3, 0, area, center);
    moments[kM03] = scale_invariant_moment(begin, end, 0, 3, area, center);

    return moments;
}

} // namespace math
} // namespace vulcan

#endif // MATH_MOMENTS_H
