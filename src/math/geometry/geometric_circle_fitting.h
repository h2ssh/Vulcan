/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     geometric_circle_fitting.h
* \author   Collin Johnson
* 
* Declaration of circle_fit_by_lm_full and circle_fit_by_lma.
* 
* Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
* 
*   - CircleFitByLevenbergMarquardtFull.cpp
*   - CircleFitByChernovLesort.cpp
*/

#ifndef MATH_GEOMETRY_GEOMETRIC_CIRCLE_FITTING_H
#define MATH_GEOMETRY_GEOMETRIC_CIRCLE_FITTING_H

#include <math/geometry/circle.h>
#include <math/geometry/circle_fitting_utils.h>

namespace vulcan
{
namespace math
{
namespace detail
{

/**
* circle_fit_by_lm_full is the Levenberg-Marquardt Full algorithm implemented by Chernov and adapted for use with Vulcan
* data types.
* 
* This function is faster than LMA, but doesn't work quite as well on small data samples.
* 
* \param    begin           Start of point data
* \param    end             End of point data
* \param    circleIni       Initial guess at circle fit (use circle_fit_by_taubin)
* \param    lamdaIni        Initial lambda step (0.01 works well)
* \param[out]   fitCircle   Circle fit to the data
* \return   Result code: 0 = normal
*                        1 = hit the upper limit of outer iterations (99)
*                        2 = hit the upper limit of inner iterations (99)
*                        3 = coordinate for center are too large
*                        4 = found zero-radius circle
*/
int circle_fit_by_lm_full(DataIterator   begin, 
                          DataIterator   end, 
                          Circle<float>& circleIni, 
                          double         lambdaIni, 
                          Circle<float>& fitCircle);

/**
* circle_fit_by_lma is the Chernov-Lesort algorithm implemented by Chernov and adapted for use with Vulcan
* data types.
* 
* This function is more likely to converge than lm_full, but is sometimes slower.
* 
* \param    begin           Start of point data
* \param    end             End of point data
* \param    circleIni       Initial guess at circle fit (use circle_fit_by_taubin)
* \param    lamdaIni        Initial lambda step (0.01 works well)
* \param[out]   fitCircle   Circle fit to the data
* \return   Result code: 0 = normal
*                        1 = hit the upper limit of outer iterations (99)
*                        2 = hit the upper limit of inner iterations (99)
*                        3 = coordinate for center are too large
*                        4 = found zero-radius circle
*/
int circle_fit_by_lma(DataIterator   begin, 
                      DataIterator   end, 
                      Circle<float>& circleIni, 
                      double         lambdaIni, 
                      Circle<float>& fitCircle);

}
}
}

#endif // MATH_GEOMETRY_GEOMETRIC_CIRCLE_FITTING_H
