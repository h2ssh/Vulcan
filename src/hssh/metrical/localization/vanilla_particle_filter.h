/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vanilla_particle_filter.h
* \author   Collin Johnson
* 
* Declaration of VanillaParticleFilter.
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_VANILLA_PARTICLE_FILTER_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_VANILLA_PARTICLE_FILTER_H

#include "hssh/metrical/localization/particle_sampler.h"
#include "hssh/metrical/localization/sample_set_distribution_calculator.h"
#include "hssh/metrical/localization/particle.h"

namespace vulcan
{
namespace hssh
{
    
const std::string VANILLA_PARTICLE_FILTER_TYPE("vanilla");
    
/**
* LowDispersionSampler samples from particles using a low-dispersion sampling approach to assume that
* the samples are not unduly favoring low-weight sample due to random effects.
*/
class LowDispersionSampler : public ParticleSampler
{
public:
    
    // ParticleSampler interface
    virtual void drawSamplesFromPrior(std::size_t                    numSamplesToDraw, 
                                      const particle_distribution_t& prior, 
                                      std::vector<particle_t>&       newSamples) const override;
};

/**
* WeightedDistributionCalculator calculates the distribution for the samples using a simple weighted
* mean and weighted covariance using the particles' weights.
*/
class WeightedDistributionCalculator : public SampleSetDistributionCalculator
{
public:
    
    // SampleSetDistributionCalculator interface
    virtual MultivariateGaussian calculateGaussianForSamples(const std::vector<particle_t>& samples) const override;
    
private:
    
    mutable std::vector<particle_t> normalizedSamples_;
};
    
}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_VANILLA_PARTICLE_FILTER_H
