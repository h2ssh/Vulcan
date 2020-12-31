/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     covariance_based_scan_weighter.h
 * \author   Collin Johnson
 *
 * Declaration of CovarianceBasedScanWeighter.
 */

#ifndef HSSH_UTILS_COVARIANCE_BASED_SCAN_WEIGHTER_H
#define HSSH_UTILS_COVARIANCE_BASED_SCAN_WEIGHTER_H

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
 * scan_weight_type_t defines the types of weighting that can be performed based on the covariance
 * of the pose estimate. See CovarianceBasedScanWeighter for an in-depth description.
 */
enum scan_weight_type_t
{
    WEIGHT_ORTHOGONAL_TO_EIGENVECTORS,
    WEIGHT_GRID_UNCERTAINTY
};

/**
 * CovarianceBasedScanWeighter calculates a weight for each ray in a laser scan. The weight can be calculated in
 * the following ways:
 *
 *   WEIGHT_ORTHOGONAL_TO_EIGENVECTORS:
 *       To determine the weight, the eigenvalue decomposition of the (x,y) covariance matrix is calculated.
 *       The weight given to a particular measurement is:
 *
 *           w = lambda_1*e_1'l_i + lambda_2*e_2'l_i
 *
 *       where lambda_n = eigenvalue, e_n = eigenvector, l_i = normalized line associated with a ray.
 *
 *       If no line is associated with a ray, the weight given is (lambda_1+lambda_2) / 2
 *
 *   WEIGHT_GRID_UNCERTAINTY:
 *       To determine the weight, the x, y, and theta variances are each considered....
 */
class CovarianceBasedScanWeighter
{
public:
    /**
     * Constructor for CovarianceBasedScanWeighter.
     *
     * \param    type            Type of weighting to be performed
     */
    CovarianceBasedScanWeighter(scan_weight_type_t type);

    /**
     * calculateWeights calculates the weights for the scan based on one of the above formulas.
     *
     * \param    scan                Scan to be weighted
     * \param    poseDistribution    Probability distribution for the pose
     * \return   Vector of scan weights. One for each ray.
     */
    std::vector<float> calculateWeights(const laser::laser_scan_lines_t& scan,
                                        const MultivariateGaussian& poseDistribution);

private:
    void calculateDecomposition(const MultivariateGaussian& priorPoseDistribution);
    void calculateScanWeights(const laser::laser_scan_lines_t& scan, const MultivariateGaussian& priorPoseDistribution);

    Vector eigenvalues;
    Matrix eigenvectors;

    std::vector<float> scanWeights;

    cartesian_laser_scan_t cartesian;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_COVARIANCE_BASED_SCAN_WEIGHTER_H
