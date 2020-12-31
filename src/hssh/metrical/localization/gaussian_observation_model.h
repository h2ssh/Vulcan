/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gaussian_observation_model.h
 * \author   Collin Johnson
 *
 * Declaration of GaussianObservationModel.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_GAUSSIAN_OBSERVATION_MODEL_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_GAUSSIAN_OBSERVATION_MODEL_H

#include "hssh/metrical/localization/observation_model.h"

namespace vulcan
{
namespace hssh
{

struct gaussian_observation_model_params_t;


const std::string kGaussianModelType("gaussian");

/**
 * GaussianObservationModel is an observation model that creates a small Gaussian blob of radius 1 centered around
 * the endpoint of the ray. The Gaussian is convolved with the grid for centered at each ray endpoint to determine the
 * likelihood of the laser scan.
 *
 * This observational model produces a less peaked distribution that the normal endpoint observation model. This model
 * is particularly suited for relocalizing because it provides a flatter distribution that doesn't require an already
 * precise pose estimate to create suitable particles for sampling.
 */
class GaussianObservationModel : public ObservationModel
{
public:
    /**
     * Constructor for GaussianObservationModel.
     *
     * \param    params          Parameters controlling the model
     */
    GaussianObservationModel(const gaussian_observation_model_params_t& params);

    // ObservationModel interface
    virtual void initializeModel(const laser::laser_scan_lines_t& scan,
                                 const MultivariateGaussian& proposalDistribution);
    virtual double sampleLikelihood(const particle_t& sample,
                                    const laser::laser_scan_lines_t& scan,
                                    const OccupancyGrid& map,
                                    particle_filter_debug_info_t* debug);

private:
    const float kMaxDistance_;
    const int kStride_;
    std::array<double, 9> gaussian_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_GAUSSIAN_OBSERVATION_MODEL_H
