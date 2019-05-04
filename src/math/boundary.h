/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary.h
* \author   Collin Johnson
*
* Declaration of functions for boundary probability distributions:
*
*   - heading_uncertainty : compute heading uncertainty from velocity uncertainty
*   - boundary_heading_range : compute range of headings occupied by a boundary relative to the robot
*   - boundary_heading_probability : compute the probability of a boundary given the robot heading and uncertainty
*/

#ifndef MATH_BOUNDARY_H
#define MATH_BOUNDARY_H

#include <math/angle_range.h>
#include <core/line.h>
#include <core/matrix.h>
#include <core/point.h>

namespace vulcan
{
namespace math
{

/**
* heading_uncertainty computes the heading uncertainty from the uncertain velocity measurements of (v_x, v_y). The
* heading for this velocity is atan2(v_y, v_x). Propagating the uncertainty is approximated using the first-order
* Jacobian yields:
*
*       sigma^2_theta = J^T (sigma_xx sigma_xy; sigma_xy sigma_yy) J
*
*   with J = [ (-v_y / (v_x^2+v_y^2) (v_x / (v_x^2+v_y^2) ]^T
* *
* \param    vx      Mean velocity estimate along x-axis
* \param    vy      Mean velocity estimate along y-axis
* \param    sigma   Covariance of the velocity estimate
* \pre  |v_x| + |v_y| > 0   (heading not defined if no velocity!)
* \return   Uncertainty of heading estimate
*/
double heading_uncertainty(double vx, double vy, const Matrix& sigma);

/**
* boundary_heading_range computes the range of headings occupied by a single boundary relative to the heading of the
* object. Thus, an angle of 0 indicates the heading of the object.
*
* \param    boundary        Boundary for which to compute the range
* \param    position        Position of the object
* \param    heading         Heading of the object
* \return   Range of angles occupied by the particular boundary.
*/
angle_range_t boundary_heading_range(const Line<double>& boundary,
                                     const Point<double>& position,
                                     double heading);

/**
* boundary_heading_probability computes the probability of a given angle range when provided with the uncertainty of the
* object's heading.
*
* The probability is computed using a truncated Gaussian distribution that computes probabilities in the range [-pi,pi].
* For angles that wrap around, the probability is the sum of [-pi,negangle] + [pi,posangle], as there's some
* probability associated with turning left or right.
*
* \param    range           Range of angles occupied by the boundary
* \param    sigma           Std dev of the object heading
* \pre  range.extent <= M_PI
* \pre  sigma > 0.0
* \return   Probability of this range of angles being visited by the object
*/
double boundary_heading_probability(const angle_range_t& range, double sigma);

} // namespace math
} // namespace vulcan

#endif // MATH_BOUNDARY_H
