/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_REGRESSION_H
#define MATH_REGRESSION_H

#include "core/line.h"
#include <cassert>
#include <cmath>
#include <numeric>
#include <vector>

namespace vulcan
{
namespace math
{

/**
* line_from_least_squares creates a line from the parameters estimated by the least-squares process.
*/
Line<double> line_from_least_squares(Point<double> p1,
                                     Point<double> p2,
                                     double xAverage,
                                     double yIntercept,
                                     double slope);

/**
* dist_to_line calculates the distance to a line based on the parameters from total least squares.
*/
double dist_to_line(Point<double> point, double a, double b);

/**
* simple_least_sqaures performs a simple least-squares fitting on the provided values, returning the
* values for the line of best-fit, C + Dt. The values are put in the pair such that:
*
*   C = retVal.first
*   D = retVal.second
*
* The inputs are the t-values on the left-hand side of the Ax=b expression and the b-values are the
* values on the right-hand side of the expression.
*
* For a Point, it is assumed that the x-values are equivalent to the t-values and the y-values are equivalent
* to the b-values.
*
* For the fit to be valid, begin <= end.
*
* The set size is assumed to be:  [begin, end)
*/
Line<float> simple_least_squares(std::vector<Point<float>>::const_iterator begin, std::vector<Point<float>>::const_iterator end);

/**
* total_least_squares performs a total least-squares analysis on a set of points.
*/
template <class PointIterator>
Line<double> total_least_squares(PointIterator begin, PointIterator end)
{
    /*
    *   B = 0.5 * (sum(y^2) - n * avg(y)^2 - (sum(x^2) - n * avg(x)^2)) / (n * avg(x) * avg(y) - sum(x * y))
    *   b = -B +- sqrt(B^2 + 1)
    *   a = avg(y) - b * avg(x)
    *
    * The above equation comes from: http://mathworld.wolfram.com/LeastSquaresFittingPerpendicularOffsets.html
    */

    using PointType = decltype(*begin);

    int numPoints = std::distance(begin, end);

    // ERROR: Can't define a line with less than two points!
    if(numPoints < 2)
    {
        assert(numPoints < 2);
        return Line<double>();
    }

    // When accumulating, cast one of the multiplies to double to ensure the point value doesn't overflow!
    double avgY  = std::accumulate(begin, end, 0.0, [](double total, const PointType& p) { return total + p.y;     }) / numPoints;
    double sumY2 = std::accumulate(begin, end, 0.0, [](double total, const PointType& p) { return total + static_cast<double>(p.y)*p.y; });
    double avgX  = std::accumulate(begin, end, 0.0, [](double total, const PointType& p) { return total + p.x;     }) / numPoints;
    double sumX2 = std::accumulate(begin, end, 0.0, [](double total, const PointType& p) { return total + static_cast<double>(p.x)*p.x; });
    double sumXY = std::accumulate(begin, end, 0.0, [](double total, const PointType& p) { return total + static_cast<double>(p.y)*p.x; });

    /*
    * NOTE: There are two potential lines that are created by this method for the regression, so which should be used?
    *       My strategy is to generate the line for both possibilities and then choose the one that minimizes the distance
    *       to the start and end points.
    */

    double yErr  = sumY2 - numPoints*avgY*avgY;
    double xErr  = sumX2 - numPoints*avgX*avgX;
    double denom = numPoints*avgX*avgY - sumXY;

    std::pair<double, double> line;

    if(yErr == 0)  // yErr == 0 means that we're dealing with a flat line
    {
        line.first  = avgY;
        line.second = 0;
    }
    else if((xErr == 0) || (denom == 0)) // xErr == 0 means that the line is vertical. denom = 0 gives line of infinite slope as well
    {
        line.first  = 0;
        line.second = INFINITY;
    }
    else
    {
        double B  = 0.5 * (yErr - xErr) / denom;
        double b1 = -B + std::sqrt(B * B + 1);
        double a1 = avgY - b1 * avgX;
        double b2 = -B - std::sqrt(B * B + 1);
        double a2 = avgY - b2 * avgX;

        if((dist_to_line(*begin, a1, b1) + dist_to_line(*(end - 1), a1, b1)) < (dist_to_line(*begin, a2, b2) + dist_to_line(*(end - 1), a2, b2)))
        {
            line.first  = a1;
            line.second = b1;
        }
        else
        {
            line.first  = a2;
            line.second = b2;
        }
    }

    return line_from_least_squares(*begin, *(end-1), avgX, line.first, line.second);
}

/** correlation finds the correlation/r-values amongst a series of data points. */
template <class PointIterator>
double correlation(PointIterator begin, PointIterator end)
{
    /**
    * The correlation/r-value is calculated via the following:
    *
    *   corr = sum((x - avgX)(y - avgY)) / sqrt(sum((x - avgX)^2 * (y - avgY)^2))
    */

    // No points? Completely uncorrelated
    if(begin == end)
    {
        return 0.0;
    }

    double avgX     = 0;
    double avgY     = 0;
    double sumX2    = 0;
    double sumY2    = 0;
    double sumNumer = 0;

    int numPoints = 0;

    for(auto pointIt = begin; pointIt != end; ++pointIt)
    {
        avgX += pointIt->x;
        avgY += pointIt->y;

        ++numPoints;
    }

    avgX /= numPoints;
    avgY /= numPoints;

    // Now sum the difference of all the points and call it a day

    for(auto pointIt = begin; pointIt != end; ++pointIt)
    {
        sumNumer += (pointIt->x - avgX) * (pointIt->y - avgY);
        sumX2    += std::pow(pointIt->x - avgX, 2);
        sumY2    += std::pow(pointIt->y - avgY, 2);
    }

    return (sumX2*sumY2 != 0.0) ? sumNumer / std::sqrt(sumX2*sumY2) : 0.0;
}

}
}

#endif // MATH_REGRESSION_H
