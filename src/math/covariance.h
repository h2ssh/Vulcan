/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     covariance.h
* \author   Collin Johnson
*
* Definition of functions for dealing with the covariance of collections of points:
*
*   - point2D_covariance            : calculate covariance of a collection of points
*   - point2D_covariance_properties : calculate direction and eigenvalue ratio of a collection of points
*/

#ifndef MATH_COVARIANCE_H
#define MATH_COVARIANCE_H

#include <core/matrix.h>
#include <core/vector.h>
#include <core/angle_functions.h>

namespace vulcan
{
namespace math
{

/**
* point_2d_covariance_properties_t holds two properties about the covariance matrix of a collection of 2d points:
*
*   - direction     : the direction the largest eigenvector points
*   - eccentricity  : 1.0 - min/max
*/
struct point2D_covariance_properties_t
{
    double direction;
    double eccentricity;
};

/**
* point2D_covariance calculates the covariance of a collection of 2D points. Each point is
* expected to have public x and y fields. [startPoint, endPoint) is calculated.
*
* \param    startPoint          Starting point of values to calculate
* \param    endPoint            End of range to calculate
* \param    startWeight         Start weight for the weights to apply to the points. Must be valid for dist(start, end).
* \return   2x2 covariance matrix of the values.
*/
template <typename PointIterator, typename WeightIterator>
Matrix point2D_covariance(PointIterator startPoint, PointIterator endPoint, WeightIterator startWeight)
{
    // Calculate the mean, then the values for the covariance matrix
    double meanX = 0;
    double meanY = 0;
//     int    count = 0;

    WeightIterator weightIt = startWeight;
    double sumWeights = 0.0;

    for(PointIterator pointIt = startPoint; pointIt != endPoint; ++pointIt, ++weightIt)
    {
        meanX += pointIt->x * (*weightIt);
        meanY += pointIt->y * (*weightIt);
        sumWeights += *weightIt;
    }

    sumWeights = std::max(sumWeights, 1.0);

    meanX /= sumWeights;
    meanY /= sumWeights;

    double sigmaXX = 0;
    double sigmaYY = 0;
    double sigmaXY = 0;
    double sumSqWeights = 0.0;

    weightIt = startWeight;

    for(PointIterator pointIt = startPoint; pointIt != endPoint; ++pointIt, ++weightIt)
    {
        sigmaXX += (pointIt->x - meanX) * (pointIt->x - meanX) * (*weightIt);
        sigmaYY += (pointIt->y - meanY) * (pointIt->y - meanY) * (*weightIt);
        sigmaXY += (pointIt->x - meanX) * (pointIt->y - meanY) * (*weightIt);

        sumSqWeights += (*weightIt) * (*weightIt);
    }

    // Normalize with this value to get the unbiased variance estimate is described here:
    // http://en.wikipedia.org/wiki/Weighted_mean
    double denom = (sumWeights * sumWeights) - sumSqWeights;
    double normalizer = (denom > 0.0) ? 1.0 / denom : 1.0;

    Matrix covariance(2, 2);
    covariance(0, 0) = sigmaXX * normalizer;
    covariance(0, 1) = sigmaXY * normalizer;

    covariance(1, 0) = sigmaXY * normalizer;
    covariance(1, 1) = sigmaYY * normalizer;

    return covariance;
}


/**
* point2D_covariance_properties finds the direction of the major axis eigenvector and the eccentricity of the points.
*
* \param    startPoint          Start of the points
* \param    endPoint            One-past the end of the points
* \param    startWeight         Starting iterator for weights. Valid range must be equal to dist(start,end)
* \return   Properties of covariance -- direction and eccentricity
*/
template <typename PointIterator, typename WeightIterator>
point2D_covariance_properties_t point2D_covariance_properties(PointIterator startPoint,
                                                              PointIterator endPoint,
                                                              WeightIterator startWeight)
{
    auto covariance = point2D_covariance(startPoint, endPoint, startWeight);

    Vector eigenvalues;
    Matrix eigenvectors;

    arma::eig_sym(eigenvalues, eigenvectors, covariance);

    double eccentricty = 0.0;

    if(eigenvalues(1) == 0.0)
    {
        eccentricty = 0.0;
    }
    else
    {
        eccentricty = std::max(1.0 - eigenvalues(0)/eigenvalues(1), 0.0);
    }

    double direction = wrap_to_pi_2(std::atan2(eigenvectors(1, 1), eigenvectors(1, 0)));

    return { direction, eccentricty };
}

}
}

#endif // MATH_COVARIANCE_H
