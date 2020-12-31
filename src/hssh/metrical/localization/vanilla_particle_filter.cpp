/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vanilla_particle_filter.cpp
* \author   Collin Johnson
* 
* Definition of VanillaParticleFilter.
*/

#include "hssh/metrical/localization/vanilla_particle_filter.h"
#include "hssh/metrical/localization/particle_filter_utils.h"

namespace vulcan 
{
namespace hssh
{

void LowDispersionSampler::drawSamplesFromPrior(std::size_t                    numSamplesToDraw, 
                                                const particle_distribution_t& prior, 
                                                std::vector<particle_t>&       newSamples) const
{
    draw_samples_from_cdf(prior.samples, numSamplesToDraw, newSamples);
}


MultivariateGaussian WeightedDistributionCalculator::calculateGaussianForSamples(const std::vector<particle_t>& samples) const
{
    auto maxSample = std::max_element(samples.begin(), samples.end());
    
    Vector maxVector(3);
    maxVector(0) = maxSample->pose.x;
    maxVector(1) = maxSample->pose.y;
    maxVector(2) = maxSample->pose.theta;
    
    Matrix cov = calculate_sample_set_covariance(samples, maxVector);
    return MultivariateGaussian(maxVector, cov);
}

}
}
