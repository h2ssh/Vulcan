/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     interpolation.cpp
* \author   Collin Johnson
*
* Definition of interpolation functions declared in interpolation.h.
*/

#include "math/interpolation.h"
#include "core/point.h"
#include <cassert>

namespace vulcan
{
namespace math
{

float unit_bilinear_interpolation(const Point<float>& point, float values[4])
{
    assert(0 <= point.x && point.x <= 1 && 0 <= point.y && point.y <= 1);

    return (values[0] * (1.0f-point.x) * (1.0f-point.y)) +
           (values[1] * point.x        * (1.0f-point.y)) +
           (values[2] * (1.0f-point.x) * point.y)        +
           (values[3] * point.x        * point.y);
}

}
}
