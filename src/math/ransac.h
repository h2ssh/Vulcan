/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ransac.h
* \author   Collin Johnson
*
* Declaration of a variety of RANSAC-based fitting algorithms.
*/

#ifndef MATH_RANSAC_H
#define MATH_RANSAC_H

#include <vector>
#include <core/line.h>

namespace vulcan
{
namespace math
{

struct ransac_line_t
{
    Line<float> line;
    size_t            numInliers;

    ransac_line_t(void)
            : numInliers(0)
    {
    }

    ransac_line_t(const Line<float>& line, size_t inliers)
            : line(line)
            , numInliers(inliers)
    {
    }
};

/**
* ransac_single_line uses RANSAC to extract a single line estimate from a set of points. The parameters
* to set are:
*
*   - inlier distance       : distance from model line for a point to be considered an inlier
*   - number of iterations  : maximum number of iterations to run
*
* \param    points              Points from which to extract a line
* \param    numIterations       Number of iterations to run RANSAC
* \param    inlierDistance      Maximum distance from the model line for inliers
* \return   The best fit line to the data.
*/
ransac_line_t ransac_single_line(const std::vector<Point<float>>& points, uint16_t numIterations, float inlierDistance);

}
}

#endif // MATH_RANSAC_H
