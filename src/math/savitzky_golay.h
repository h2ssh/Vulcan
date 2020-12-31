/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     savitzky_golay.h
 * \author   Collin Johnson
 *
 * Declaration of functions implementing Savitzky-Golay filtering for smoothing data or calculating smoothed
 * derivatives:
 *
 *   - savitzky_golay_smooth       : smooth using Savitzky-Golay
 *   - savitzky_golay_first_deriv  : smoothed first derivative of the data
 *   - savitzky_golay_second_deriv : smoothed second derivative of the data
 *
 * The Savitsky-Golay filter is described well on Wikipedia, on which this implementation is based:
 *
 *   https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter
 *
 * The filter requires an odd number of elements when performing the filtering. It works by fitting a polynomial to each
 * data point using least-squares regression. This regression has a closed-form solution, and thus the particular
 * coefficients for a window size and fitting polygon are always the same and can be encoded in a table.
 *
 * This implementation uses a table for these coefficients, so it only supports a limited number of fitting polynomials
 * and window sizes. These polynomials and window radii are encoded in the enums used with the filter.
 *
 * For each filter, there must be at least 2*window_radius + 1 data members, as that is the size of the filter window.
 */

#ifndef MATH_SAVITZKY_GOLAY_H
#define MATH_SAVITZKY_GOLAY_H

#include <vector>

namespace vulcan
{
namespace math
{

using DataIter = std::vector<double>::iterator;
using DataConstIter = std::vector<double>::const_iterator;


/**
 * SGWindowRadius defines the supported window radius values for this implementation.
 */
enum class SGWindowRadius
{
    two,
    three,
    four
};

/**
 * SGPolynomial defines the supported polynomials to fit to the data.
 */
enum class SGPolynomial
{
    quadratic,
    quartic
};


/**
 * savitzky_golay_smooth smooths the input data and stores it in the output. The radius and polynomial to fit can be
 * specified, but default to radius = 2 and quadratic polynomial.
 *
 * \param    begin           Start of the data
 * \param    end             End of the data
 * \param    output          Start of the output in which to store the smoothed data
 * \param    radius          Radius of the window (optional, default = two)
 * \param    polynomial      Polynomial to fit (optional, default = quadratic)
 */
void savitzky_golay_smooth(DataConstIter begin,
                           DataConstIter end,
                           DataIter output,
                           SGWindowRadius radius = SGWindowRadius::two,
                           SGPolynomial polynomial = SGPolynomial::quadratic);

/**
 * savitzky_golay_first_deriv calculates the first derivative of the input data and stores it in the output. The radius
 * and polynomial to fit can be specified, but default to radius = 2 and quadratic polynomial.
 *
 * \param    begin           Start of the data
 * \param    end             End of the data
 * \param    output          Start of the output in which to store the first derivatives
 * \param    radius          Radius of the window (optional, default = two)
 * \param    polynomial      Polynomial to fit (optional, default = quadratic)
 */
void savitzky_golay_first_deriv(DataConstIter begin,
                                DataConstIter end,
                                DataIter output,
                                SGWindowRadius radius = SGWindowRadius::two,
                                SGPolynomial polynomial = SGPolynomial::quadratic);

/**
 * savitzky_golay_second_deriv calculates the second derivative of the input data and stores it in the output. The
 * radius and polynomial to fit can be specified, but default to radius = 2 and quadratic polynomial.
 *
 * \param    begin           Start of the data
 * \param    end             End of the data
 * \param    output          Start of the output in which to store the second derivatives
 * \param    radius          Radius of the window (optional, default = two)
 * \param    polynomial      Polynomial to fit (optional, default = quadratic)
 */
void savitzky_golay_second_deriv(DataConstIter begin,
                                 DataConstIter end,
                                 DataIter output,
                                 SGWindowRadius radius = SGWindowRadius::two,
                                 SGPolynomial polynomial = SGPolynomial::quadratic);
}   // namespace math
}   // namespace vulcan

#endif   // MATH_SAVITZKY_GOLAY_H
