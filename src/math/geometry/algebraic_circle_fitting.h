/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     algebraic_circle_fitting.h
 * \author   Collin Johnson
 *
 * Declaration of circle_fit_by_taubin.
 *
 * Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
 *
 *   - CircleFitByTaubin.cpp
 */

#ifndef MATH_GEOMETRY_ALGEBRAIC_CIRCLE_FITTING_H
#define MATH_GEOMETRY_ALGEBRAIC_CIRCLE_FITTING_H

#include "math/geometry/circle.h"
#include "math/geometry/circle_fitting_utils.h"

namespace vulcan
{
namespace math
{
namespace detail
{

Circle<float> circle_fit_by_taubin(DataIterator begin, DataIterator end);

}
}   // namespace math
}   // namespace vulcan

#endif   // MATH_GEOMETRY_ALGEBRAIC_CIRCLE_FITTING_H
