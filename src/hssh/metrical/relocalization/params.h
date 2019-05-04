/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
* 
* Declaration of metric_relocalizer_params_t and load_metric_relocalizer_params.
*/

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_PARAMS_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_PARAMS_H

#include <hssh/metrical/localization/params.h>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace hssh
{

struct metric_relocalizer_params_t
{
    int   maxRelocalizationAttempts;        ///< Maximum number of times to update the relocalization before failing
    float maxPositionStdDev;                ///< Maximum variance of position estimate for successful relocalization
    float maxOrientationStdDev;             ///< Maximum variance of orientation estimate for successful relocalization
    
    int minParticleFilterSamples;           ///< Minimum number of particles for each sampling step -- should be a reasonably large number (>50)
    
    std::string filterType;                 ///< Particle filter implementation to be used
    std::string motionModelType;            ///< Motion model to use in the particle filter
    std::string observationModelType;       ///< Observation model for weighting the particles
    
    particle_filter_params_t   filterParams;
    motion_model_params_t      motionParams;
    observation_model_params_t observationParams;
    
    metric_relocalizer_params_t(const utils::ConfigFile& config);
};

}
}

#endif // HSSH_UTILS_METRICAL_RELOCALIZATION_PARAMS_H
