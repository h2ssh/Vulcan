/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     float_comparison.h
* \author   Collin Johnson
*
* float_comparison.h contains functions for doing fuzzy comparisons of floating
* points numbers. These functions are useful for operations sensitive to floating
* point error.
*/

#ifndef UTILS_FLOAT_COMPARISON_H
#define UTILS_FLOAT_COMPARISON_H

#include <cmath>

namespace vulcan
{

const double MAX_ABSOLUTE_ERROR_FLOAT = 0.0001;
const double MAX_RELATIVE_ERROR_FLOAT = 0.0001;

/**
* absolute_fuzzy_equal compares two floating point for absolute error. Absolute
* error means the difference between two values is within a fixed threshold.
*
* See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm for
* fantastic detail.
*/
template <typename T, typename U>
inline bool absolute_fuzzy_equal(T x, U y, double error = MAX_ABSOLUTE_ERROR_FLOAT)
{
    return fabs(x-y) < error;
}

/**
* relative_fuzzy_equal compares the relative error between two floating point
* numbers. If the difference is less than some threshold, the numbers are considered
* to be equal.
*
* See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm for
* fantastic detail.
*/
template <typename T, typename U>
inline bool relative_fuzzy_equal(T x, U y, double error = MAX_RELATIVE_ERROR_FLOAT)
{
    if(x == y)
    {
        return true;
    }

    return (fabs(x-y)/y <= error) ||
           (fabs(x-y)/x <= error);
}

/**
* round_in_tolerance takes the ceiling a value if it is within some tolerance of the next integer. Otherwise, the
* floor is returned.
*/
template <typename T>
inline T round_in_tolerance(T value, double error = MAX_ABSOLUTE_ERROR_FLOAT)
{
    T ceiling = ceil(value);
    if(std::abs(value - ceiling) < MAX_ABSOLUTE_ERROR_FLOAT)
    {
        return ceiling;
    }

    return floor(value);
}

} // namespace vulcan

#endif // UTILS_FLOAT_COMPARISON_H
