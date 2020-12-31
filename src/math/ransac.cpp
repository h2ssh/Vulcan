/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ransac.cpp
* \author   Collin Johnson
*
* Definition of various RANSAC-based fitting algorithms.
*/

#include <cmath>
#include <iostream>
#include "math/ransac.h"
#include "math/regression.h"

namespace vulcan
{
namespace math
{

ransac_line_t ransac_single_line(const std::vector<Point<float>>& points, uint16_t numIterations, float inlierDistance)
{
    const size_t INITIAL_MODEL_SIZE = 5;

    if(points.size() < INITIAL_MODEL_SIZE)
    {
        return ransac_line_t();
    }

    Line<float> bestLine;
    size_t            maxInliers = 0;

    std::vector<Point<float>> modelPoints(INITIAL_MODEL_SIZE);
    Line<float>                modelLine;
    std::vector<Point<float>> inliers;

    do
    {
        inliers.clear();

        for(size_t n = 0; n < INITIAL_MODEL_SIZE; ++n)
        {
            modelPoints[n] = points[round(drand48()*(points.size()-1))];
        }

        modelLine = total_least_squares(modelPoints.begin(), modelPoints.end());

        // Sanity check the modelLine because the points are drawn with replacement, so there is some likelihood that
        // all modelPoints are the same, which breaks the line-fitting. A broken line fitting will have the endpoints
        // being the same
        if(modelLine.a == modelLine.b)
        {
            continue;
        }

        for(size_t n = 0; n < points.size(); ++n)
        {
            if(distance_to_line(points[n], modelLine) <= inlierDistance)
            {
                inliers.push_back(points[n]);
            }
        }

        if(inliers.size() < INITIAL_MODEL_SIZE)
        {
            continue;
        }

        modelLine = total_least_squares(inliers.begin(), inliers.end());

        size_t numInliers = 0;
        for(size_t n = 0; n < points.size(); ++n)
        {
            if(distance_to_line(points[n], modelLine) <= inlierDistance)
            {
                ++numInliers;
            }
        }

        if(numInliers > maxInliers)
        {
            bestLine   = modelLine;
            maxInliers = numInliers;
        }
    } while(--numIterations > 0);

    return ransac_line_t(bestLine, maxInliers);
}

}
}
