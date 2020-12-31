/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     scan_weighting.h
 * \author   Collin Johnson
 *
 * Declaration of functions that calculate various types of weights for scans:
 *
 *   - calculate_covariance_weights : scan points orthogonal to highest uncertainty in proposal distribution receive
 *                                    higher weights
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_SCAN_WEIGHTING_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_SCAN_WEIGHTING_H

#include <vector>

namespace vulcan
{

class MultivariateGaussian;

namespace laser
{
struct laser_scan_lines_t;
}
namespace hssh
{

/**
 * calculate_covariance_weights
 *
 * \param[in]    scan                    Scan with calculated lines to be weighted
 * \param[in]    proposalDistribution    Proposal distribution that defines estimated uncertainty of robot pose
 * \param[in]    stride                  Stride to take through the rays in the scan
 * \param[out]   weights                 Calculated covariance weights -- size == (scan.numRanges / stride)
 */
void calculate_covariance_weights(const laser::laser_scan_lines_t& scan,
                                  const MultivariateGaussian& proposalDistribution,
                                  int stride,
                                  std::vector<float>& weights);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_SCAN_WEIGHTING_H
