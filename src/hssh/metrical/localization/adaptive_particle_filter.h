/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     adaptive_particle_filter.h
* \author   Collin Johnson
*
* Declaration of a ParticleSampler and SampleSetDistributionCalculator that make the ParticleFilter
* implement some of the ideas from the Beeson, et. al ICRA '06 paper:
* 
*   Adapting Proposal Distributions for Accurate, Efficient Mobile Robot Localization
*
* A copy can be found here:
*
* http://www.eecs.umich.edu/~kuipers/research/pubs/Beeson-etal-icra-06.html
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_ADAPTIVE_PARTICLE_FILTERS_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_ADAPTIVE_PARTICLE_FILTERS_H

#include <hssh/metrical/localization/particle_sampler.h>
#include <hssh/metrical/localization/sample_set_distribution_calculator.h>
#include <hssh/metrical/localization/particle.h>
#include <string>

namespace vulcan
{
namespace hssh
{

const std::string ADAPTIVE_PARTICLE_FILTER_TYPE("adaptive");


/**
* GaussianParticleSampler draws samples from the Gaussian approximation of the particle distribution.
*/
class GaussianParticleSampler : public ParticleSampler
{
public:

    // ParticleSampler interface
    virtual void drawSamplesFromPrior(std::size_t                    numSamplesToDraw,
                                      const particle_distribution_t& prior,
                                      std::vector<particle_t>&       newSamples) const override;
};


/**
* BestSamplesDistributionCalculator calculates the approximate distribution for a sample set by creating
* a histogram of the particle weights and then finding the peak of the best particles. These particles
* are used for calculating the mean of the approximation. The entire sample set is then used for calculating
* the covariance.
*/
class BestSamplesDistributionCalculator : public SampleSetDistributionCalculator
{
public:
    
    // SampleSetDistributionCalculator interface
    virtual MultivariateGaussian calculateGaussianForSamples(const std::vector<particle_t>& samples) const override;
    
private:

    mutable std::vector<particle_t> filteredSamples_;
    mutable std::vector<particle_t> normalizedSamples_;
};


}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_ADAPTIVE_PARTICLE_FILTERS_H
