/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     circle_fitting_utils.h
* \author   Collin Johnson
* 
* Declaration of utility values for the circle fitting algorithms.
* 
* Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
* 
*   - Utilities.cpp
*   - mystuff.h
*   - data.h
*/

#ifndef MATH_GEOMETRY_CIRCLE_FITTING_UTILS_H
#define MATH_GEOMETRY_CIRCLE_FITTING_UTILS_H

#include "core/point.h"
#include "math/geometry/circle.h"
#include <limits>
#include <vector>
#include <cmath>

namespace vulcan
{
namespace math
{
namespace detail
{
    
using DataIterator = std::vector<Point<float>>::const_iterator;

//   next define some frequently used constants:

const double One=1.0,Two=2.0,Three=3.0,Four=4.0,Five=5.0,Six=6.0,Ten=10.0;
//const double One=1.0L,Two=2.0L,Three=3.0L,Four=4.0L,Five=5.0L,Six=6.0L,Ten=10.0L;
const double Pi=3.141592653589793238462643383L;
const double REAL_MAX=std::numeric_limits<double>::max();
const double REAL_MIN=std::numeric_limits<double>::min();
const double REAL_EPSILON=std::numeric_limits<double>::epsilon();

//   next define some frequently used functions:
Point<double> meanPoint(DataIterator begin, DataIterator end);

//****************** Sigma ************************************
//
//   estimate of Sigma = square root of RSS divided by N
//   gives the root-mean-square error of the geometric circle fit

double Sigma(DataIterator begin, DataIterator end, const Circle<float>& circle);

}
}
}

#endif // MATH_GEOMETRY_CIRCLE_FITTING_UTILS_H
