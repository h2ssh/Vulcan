/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_based_observation_models.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of ObservationModels for occupancy-grid type maps.
 */

#ifndef HSSH_UTILS_METRICAL_LPM_GRID_BASED_OBSERVATION_MODELS_H
#define HSSH_UTILS_METRICAL_LPM_GRID_BASED_OBSERVATION_MODELS_H

#include "core/laser_scan.h"
#include "core/matrix.h"
#include "core/vector.h"
#include "hssh/metrical/localization/observation_model.h"
#include "hssh/metrical/localization/params.h"
#include "math/exponential_distribution.h"
#include "math/uniform_distribution.h"
#include "math/univariate_gaussian.h"
#include <cstdint>
#include <string>

namespace vulcan
{
namespace hssh
{

// Definition of the string descriptions of the various observation models
const std::string ENDPOINT_OBSERVATION_MODEL_TYPE("endpoint");
const std::string BEAM_OBSERVATION_MODEL_TYPE("beam");

/**
 * EndpointObservationModel is a grid-based observation model that weights the value of each
 * scan based on the amount of information it carries relative to the covariance of the prior pose
 * distribution. The idea is cells orthogonal to the axes of uncertainty provide a strong constraint
 * on the distribution of values along that axis, whereas cells parallel to the axis provide less
 * information.
 *
 * To determine the weight, the eigenvalue decomposition of the (x,y) covariance matrix is calculated.
 * The weight given to a particular measurement is:
 *
 *    w = lambda_1*e_1'l_i + lambda_2*e_2'l_i
 *
 * where lambda_n = eigenvalue, e_n = eigenvector, l_i = normalized line associated with a measurement.
 *
 * If no line is associated with a measure, the weight given is (lambda_1+lambda_2) / 2
 */
class EndpointObservationModel : public ObservationModel
{
public:
    /**
     * Constructor for EndpointObservationModel.
     */
    EndpointObservationModel(const endpoint_observation_model_params_t& params);

    ////////////// ObservationModel interface //////////////////
    virtual void initializeModel(const laser::laser_scan_lines_t& scan,
                                 const MultivariateGaussian& proposalDistribution) override;

    virtual double sampleLikelihood(const particle_t& sample,
                                    const laser::laser_scan_lines_t& scan,
                                    const OccupancyGrid& map,
                                    particle_filter_debug_info_t* debug) override;

private:
    std::vector<float> scanWeights;
    std::vector<float> dx;
    std::vector<float> dy;
    const bool useScanWeights;
    const bool useMovingScan;
    const float maxLaserDistance;

    const int stride;

    inline double hitLikelihood(Point<int> cell, const OccupancyGrid& map);
};

/**
 * BeamObservationModel is a grid-based observation model that uses ray-casting and occupancy-based
 * weighting to determine how well a given scan matches the map.
 */
class BeamObservationModel : public ObservationModel
{
public:
    /**
     * Constructor for BeamObservationModel.
     */
    BeamObservationModel(const beam_observation_model_params_t& params);

    virtual ~BeamObservationModel(void) { }


    ////////////// ObservationModel interface //////////////////
    virtual void initializeModel(const laser::laser_scan_lines_t& scan,
                                 const MultivariateGaussian& proposalDistribution) override;

    virtual double sampleLikelihood(const particle_t& sample,
                                    const laser::laser_scan_lines_t& scan,
                                    const OccupancyGrid& map,
                                    particle_filter_debug_info_t* debug) override;

private:
    math::UnivariateGaussianDistribution pHit;
    math::ExponentialDistribution pShort;
    math::UniformDistribution pRand;
    double maxRange;

    std::vector<float> covarianceWeights;
    beam_observation_model_params_t params;


    // calculate the score for a single beam along the ray (radius,theta) emitted at offset
    double beamLikelihood(double radius, double theta, const Point<double>& laserPosition, const OccupancyGrid& map);

    double computeDistanceWeight(double range);
    double computeOccupancyWeight(const Point<int>& gridPoint, const OccupancyGrid& map);
    double computeLineWeight(const laser::laser_scan_lines_t& scan, int rayIndex, double rayAngle);
    double computeCovarianceWeight(const MultivariateGaussian& proposalDistribution, double beamAngle);

    // eigenvalue decomposition of proposal distribution
    void calculateDecomposition(const MultivariateGaussian& proposalDistribution);

    int rayStride(float scanAngularResolution);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LPM_GRID_BASED_OBSERVATION_MODELS_H
