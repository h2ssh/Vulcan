/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_sampler.h
* \author   Collin Johnson
* 
* Definition of ParticleSampler interface.
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_SAMPLER_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_SAMPLER_H

#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{
    
class  ParticleSampler;
struct particle_distribution_t;
struct particle_t;

/**
* create_particle_sampler creates a ParticleSampler of the desired type. If no subclass is associated with
* the type string, then a fatal error occurs.
* 
* \param    type            Type of ParticleSampler to create
* \return   An instance the desired subclass of ParticleSampler.
*/
std::unique_ptr<ParticleSampler> create_particle_sampler(const std::string& type);

/**
* ParticleSampler is the interface for the sampling strategy employed by the ParticleFilter. A sampler
* only needs to draw samples from a prior distribution via a call to the drawSamplesFromPrior method.
*/
class ParticleSampler
{
public:
    
    virtual ~ParticleSampler(void) { }
    
    /**
    * sampleNewParticles draws a specified number of samples from the supplied prior distribution.
    * 
    * \param[in]    numSamplesToDraw            Number of samples to draw from the prior
    * \param[in]    prior                       Prior distribution from which to draw samples
    * \param[out]   newSamples                  Location to save the drawn samples
    */
    virtual void drawSamplesFromPrior(std::size_t                    numSamplesToDraw,
                                      const particle_distribution_t& prior,
                                      std::vector<particle_t>&       newSamples) const = 0;
};
    
}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_SAMPLER_H
