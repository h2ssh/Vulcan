/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     circle_fitting_utils.cpp
* \author   Collin Johnson
*
* Definition of utility functions for the circle fitting algorithms.
*
* Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
*
*   - Utilities.cpp
*   - mystuff.h
*   - data.h
*/

#include <math/geometry/circle_fitting_utils.h>
#include <math/geometry/circle.h>
#include <algorithm>
#include <numeric>

namespace vulcan
{
namespace math
{
namespace detail
{

Point<double> meanPoint(DataIterator begin, DataIterator end)
{
    Point<double> mean;
    mean = std::accumulate(begin, end, mean);
    mean.x /= std::distance(begin, end);
    mean.y /= std::distance(begin, end);

    return mean;
}


//****************** Sigma ************************************
//
//   estimate of Sigma = square root of RSS divided by N
//   gives the root-mean-square error of the geometric circle fit

double Sigma(DataIterator begin, DataIterator end, const Circle<float>& circle)
{
    double sum = 0.0;

    std::for_each(begin, end,
                  [&](const Point<float>& p)
                  {
                      sum += std::pow(distance_between_points(p, circle.center()) - circle.radius(), 2);
                  });

    return sqrt(sum / std::distance(begin, end));
}

}
}
}
