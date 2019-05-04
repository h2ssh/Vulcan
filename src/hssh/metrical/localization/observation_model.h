/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     observation_model.h
* \author   Collin Johnson
*
* Declaration of ObservationModel abstract base class for different potential observation models and the create_observation_model() factory.
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_OBSERVATION_MODEL_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_OBSERVATION_MODEL_H

#include <memory>
#include <string>

namespace vulcan
{

class MultivariateGaussian;

namespace laser { struct laser_scan_lines_t;  }
namespace robot { struct velocity_t;          }
namespace hssh
{

struct particle_t;
struct observation_model_params_t;
struct particle_filter_debug_info_t;
class  OccupancyGrid;
class  ObservationModel;

/**
* create_observation_model is a factory that creates a new observation model based on the provided model name and parameters.
*/
std::unique_ptr<ObservationModel> create_observation_model(const std::string& modelName, const observation_model_params_t& params);

/**
* ObservationModel is an interface for the observation model used by the MCL
* algorithm.
*
* The observation model is used to weight samples in the proposal distribution
* with their likelihood in the posterior distribution.
*
* To use the ObservationModel, a two methods exists:
*
*   - void reset(void) : resets the observation when a new filter update occurs
*
*   - virtual double sampleLikelihood(const particle_t&   sample,
*                                     const laser::laser_scan_lines_t&  scan,
*                                     const MultivariateGaussian& priorPoseDistribution,
*                                     const LocalPerceptualMap&         map,
*                                     particle_filter_debug_info_t&     debug)
*
* sampleLikelihood returns a likelihood measure for how well the scan matches the map
* if it had been taken from the sample pose.
*/
class ObservationModel
{
public:

    virtual ~ObservationModel(void) { }

    /**
    * initializeModel resets the per-filter-update state of an observation model. As such, it should be called at the
    * beginning of each new filter update. It should NOT be called per sample update.
    */
    virtual void initializeModel(const laser::laser_scan_lines_t&  scan,
                                 const MultivariateGaussian& proposalDistribution) = 0;

    /**
    * sampleLikelihood returns a likelihood measure for how well the scan matches the map
    * if it had been taken from the sample pose.
    */
    virtual double sampleLikelihood(const particle_t&                 sample,
                                    const laser::laser_scan_lines_t&  scan,
                                    const OccupancyGrid&              map,
                                    particle_filter_debug_info_t*     debug) = 0;
};

}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_OBSERVATION_MODEL_H
