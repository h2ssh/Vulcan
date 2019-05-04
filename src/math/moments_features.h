/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     moments_features.h
 * \author   Collin Johnson
 *
 */

#ifndef MATH_MOMENTS_FEATURES_H
#define MATH_MOMENTS_FEATURES_H

#include <core/angle_functions.h>
#include <core/matrix.h>
#include <math/moments.h>
#include <core/vector.h>
#include <array>

namespace vulcan
{
namespace math
{

enum ShapeFeatureIndex
{
    kShapeOrientation,
    kShapeWeightedOrientation,
    kShapeEccentricity,
    kShapeCompactness,
    kNumShapeFeatures
};

enum HuFeatureIndex
{
    kHu0,
    kHu1,
    kHu2,
    kHu3,
    kHu4,
    kHu5,
    kHu6,
    kNumHuFeatures
};


using ShapeFeatures = std::array<double, kNumShapeFeatures>;
using HuFeatures    = std::array<double, kNumHuFeatures>;

/**
* shape_features calculates three features for a set of points that define some shape:
*
*   - compactness  : how circle-like the shape is
*   - eccentricity : how dominant one axis is
*   - orientation  : orientation of the ellipse representing the points
*/
template <class PointIterator>
ShapeFeatures shape_features(PointIterator begin, PointIterator end, double area, const Point<double>& center)
{
    if(area <= 0.0)
    {
        return ShapeFeatures();
    }

    // Create normalized second order moments
    auto m = second_order_central_moments(begin, end, center);
    for(auto& val : m)
    {
        val /= area;
    }

    Matrix cov(2, 2);
    cov(0, 0) = m[kM20];
    cov(1, 1) = m[kM02];
    cov(0, 1) = m[kM11];
    cov(1, 0) = m[kM11];

    Vector eigenvalues;
    Matrix eigenvectors;

    // Eigenvalues and vectors are stored in ascending order
    arma::eig_sym(eigenvalues, eigenvectors, cov);

    ShapeFeatures features;
    std::fill(features.begin(), features.end(), 0.0);
    if(eigenvalues(1) > 0.0)
    {
        features[kShapeEccentricity] = std::sqrt(std::max(1.0 - eigenvalues(0)/eigenvalues(1), 0.0));

        // Always use a left-handed coordinate system for the eigenvectors, with the largest being the axis.
        if(angle_between_points(Point<double>(eigenvectors(0, 1), eigenvectors(1, 1)),
            Point<double>(eigenvectors(0, 0), eigenvectors(1, 0)),
                                        Point<double>(0.0, 0.0)) < 0)
        {
            eigenvectors.col(0) *= -1;
        }

        Vector avgVector = eigenvectors.col(0)*eigenvalues(0) + eigenvectors.col(1)*eigenvalues(1);

        if(avgVector(0) != 0.0)
        {
            features[kShapeWeightedOrientation] = atan(avgVector(1) / avgVector(0));
        }
        else
        {
            features[kShapeWeightedOrientation] = M_PI_2;
        }

        if(eigenvectors(0, 1) != 0.0)
        {
            features[kShapeOrientation] = atan(eigenvectors(1, 1) / eigenvectors(0, 1));
        }
        else
        {
            features[kShapeOrientation] = M_PI_2;
        }

        // Polar moment of polygon:
        // https://math.stackexchange.com/questions/59470/calculating-moment-of-inertia-in-2d-planar-polygon
        double numer = 0.0;
        double denom = 0.0;

        for(auto pIt = begin; pIt != end; ++pIt)
        {
            double x = pIt->x - center.x;
            double y = pIt->y - center.y;

            double xNext = (pIt + 1 == end) ? begin->x - center.x : (pIt + 1)->x - center.x;
            double yNext = (pIt + 1 == end) ? begin->y - center.y : (pIt + 1)->y - center.y;

            double pNumer = ((x * x) + (y * y) + (x * xNext) + (y * yNext) + (xNext * xNext) + (yNext * yNext))
                * ((x * yNext) - (xNext * y));
            double pDenom = (x * yNext) - (xNext * y);

            numer += pNumer;
            denom += pDenom;
        }

        double moment = (denom != 0.0) ? (area / 6.0) * numer / denom : 0.0;
        if(moment > 0.0)
        {
            features[kShapeCompactness] = area * area / (2.0 * M_PI * moment);
        }
        else
        {
            // Compactness would have to be zero if moments are zero because shape must have zero area
            features[kShapeCompactness] = 0.0;
        }
    }

    return features;
};


/**
* hu_features calculates the seven scale, position, and rotation invariant features defined by Hu in 1962. See
* http://en.wikipedia.org/wiki/Image_moment for the details on the horrid-looking equations.
*
* These features are based on the third moment of the area.
*
* \param    begin           First point in the sequence
* \param    end             End of the point sequence
* \param    area            Area of the points
* \param    center          Center of the points
* \return   The seven HuFeatures.
*/
template <class PointIterator>
HuFeatures hu_features(PointIterator begin, PointIterator end, double area, const Point<double>& center)
{
    auto m = third_order_scale_invariant_moments(begin, end, area, center);

    HuFeatures features;

    features[kHu0] = m[kM20] + m[kM02];

    features[kHu1] = std::pow(m[kM20] - m[kM02], 2) + (4 * m[kM11] * m[kM11]);

    features[kHu2] = std::pow(m[kM30]-3.0*m[kM12], 2) + std::pow(3*m[kM21]-m[kM03], 2);

    features[kHu3] = std::pow(m[kM30]+m[kM12], 2) + std::pow(m[kM21]+m[kM03], 2);

    features[kHu4] = (m[kM30]-3.0*m[kM12]) * (m[kM30]+m[kM12]) * (std::pow(m[kM30]+m[kM12], 2) - 3.0*std::pow(m[kM21]+m[kM03], 2)) +
    (3.0*m[kM21]-m[kM03]) * (m[kM21]+m[kM03]) * (3.0*std::pow(m[kM30]+m[kM12], 2) - std::pow(m[kM21]+m[kM03], 2));

    features[kHu5] = (m[kM20] - m[kM02]) * (std::pow(m[kM30]+m[kM12], 2) - std::pow(m[kM21]+m[kM03], 2)) +
    (4.0 * m[kM11] * (m[kM30]+m[kM12]) * (m[kM21]+m[kM03]));

    features[kHu6] = (3.0*m[kM21]-m[kM03]) * (m[kM30]+m[kM12]) * (std::pow(m[kM30]+m[kM12], 2) - 3.0*(std::pow(m[kM21]+m[kM03], 2.0))) -
    (m[kM30]-3.0*m[kM12]) * (m[kM21]+m[kM03]) * (3.0*std::pow(m[kM30]+m[kM12], 2) - std::pow(m[kM21]+m[kM03], 2.0));

    return features;
};

} // namespace math
} // namespace vulcan

#endif // MATH_MOMENTS_FEATURES_H
