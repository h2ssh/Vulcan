/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     trigonometry.h
* \author   Collin Johnson
*
* Definition of some functions that fill out the trig functions in the standard library:
*
*   - cot
*   - csc
*   - sec
*/

#ifndef MATH_TRIGONOMETRY_H
#define MATH_TRIGONOMETRY_H

#include <cmath>
#include <limits>

namespace vulcan
{
namespace math
{

template <typename T>
double cot(T angle)
{
    return std::tan(M_PI_2 - angle);
}

template <typename T>
double sec(T angle)
{
    double cosAngle = std::cos(angle);

    // If cos is 0, then return infinity
    if(cosAngle == 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }
    else
    {
        return 1.0 / cosAngle;
    }
}

template <typename T>
double csc(T angle)
{
    return sec(M_PI_2 - angle);
}

}
}

#endif // MATH_TRIGONOMETRY_H
