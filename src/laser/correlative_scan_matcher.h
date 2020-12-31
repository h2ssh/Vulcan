/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     correlative_scan_matcher.h
* \author   Collin Johnson
*
* Declaration of CorrelativeScanMatcher.
*/

#ifndef LASER_CORRELATIVE_SCAN_MATCHER_H
#define LASER_CORRELATIVE_SCAN_MATCHER_H

#include "laser/scan_matcher_params.h"
#include "laser/scan_likelihood_grid.h"
#include "core/laser_scan.h"
#include "core/multivariate_gaussian.h"

namespace vulcan
{
namespace laser
{

/**
* CorrelativeScanMatcher implements the scan matching algorithm described in:
*
* http://april.eecs.umich.edu/pdfs/olson2009icra.pdf
*
* The basic idea behind the algorithm is the following:
*
* 0) Create a log-likelihood grid from a scan or other reference frame, like a map.
* 1) Create a search volume [x, y, theta]
* 2) Find the maximum likelihood transformation [delta_x, delta_y, delta_theta]
* 3) Calculate covariance estimate of the measurement
*
* At the current time, the scan matcher interface consists of two methods:
*
*   void setReferenceScan(const cartesian_laser_scan_t& reference);
*       -Set the scan to use as a reference for transform calculations performed by
*        findMostLikelyTransform()
*
*   math::MultiVariateGaussian findMostLikelyTransform(const cartesian_laser_scan_t& scan) const;
*       -Finds the mostly likely transformation between reference and scan. The ensuing transform is a Gaussian
*        with the mean as the transform.
*
*/
class CorrelativeScanMatcher
{
public:

    /**
    * Constructor for CorrelativeScanMatcher.
    */
    CorrelativeScanMatcher(const scan_matcher_params_t& params);

    /**
    * setReferenceScan sets the reference scan to be used for future calls to
    * findMostLikelyTransform().
    *
    * \param    reference       Points to be used for the reference scan
    * \param    referenceTime   Time at which the points in the scan were captured
    */
    void setReferenceScan(const std::vector<Point<float>>& reference, int64_t referenceTime);

    /**
    * findMostLikelyTransform searches for the most likely transformation between the previously
    * set reference scan and the current scan.
    *
    * \param    scan            Points to compare with the reference
    * \param    scanTime        Time at which the scan was taken
    */
    MultivariateGaussian findMostLikelyTransform(const std::vector<Point<float>>& scan, int64_t scanTime) const;

private:

    mutable ScanLikelihoodGrid fineResolutionGrid;
    mutable ScanLikelihoodGrid coarseResolutionGrid;

    float coarseResolutionMetersPerCell;

    scan_matcher_params_t params;

    int64_t referenceScanTimestamp; // full scan isn't needed, as necessary contents are in the likelihood grid
};

}
}

#endif // LASER_CORRELATIVE_SCAN_MATCHER_H
