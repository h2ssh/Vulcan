/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     adaptive_particle_filter.cpp
* \author   Collin Johnson
*
* Definition of GaussianParticleSampler and BestSamplesDistributionCalculator.
*/

#include <hssh/metrical/localization/adaptive_particle_filter.h>
#include <hssh/metrical/localization/particle_filter_utils.h>
#include <core/pose_distribution.h>

namespace vulcan
{
namespace hssh
{

float filter_samples_based_on_weight(const std::vector<particle_t>& sortedSamples,
                                     std::vector<particle_t>& filteredSamples);


void GaussianParticleSampler::drawSamplesFromPrior(std::size_t                    numSamplesToDraw,
                                                   const particle_distribution_t& prior,
                                                   std::vector<particle_t>&       newSamples) const
{
    auto cdfSamples  = numSamplesToDraw / 10;
    auto distSamples = numSamplesToDraw - cdfSamples;

    draw_samples_from_cdf(prior.samples, cdfSamples, newSamples);
    draw_samples_from_gaussian(prior.gaussianApproximation, distSamples, newSamples);
}


MultivariateGaussian BestSamplesDistributionCalculator::calculateGaussianForSamples(const std::vector<particle_t>& samples) const
{
    normalizedSamples_ = samples;
    std::sort(normalizedSamples_.begin(), normalizedSamples_.end(), std::greater<particle_t>());

    filteredSamples_.clear();
    filter_samples_based_on_weight(normalizedSamples_, filteredSamples_);

    // Want to use the collapsed mean, but the covariance should take into account all the generated
    // samples to get a better idea of the actual distribution of weights -- more data = better estimate
    // just don't want to skew the mean with the less good values
    Vector mean       = calculate_sample_set_mean      (filteredSamples_);
    Matrix covariance = calculate_sample_set_covariance(normalizedSamples_, mean);

    return MultivariateGaussian(mean, covariance);
}


float filter_samples_based_on_weight(const std::vector<particle_t>& sortedSamples,
                                     std::vector<particle_t>& filteredSamples)
{
    /*
    * The goal here is to find the highest peak in the histogram of particle weights. Search for where a peak
    * occurs and then where it ends. All samples with a weight greater than or equal to the weight threshold
    * of the peak are placed into the filtered samples.
    */

    float kWeightCutoff = 0.95;
    float cutoffWeight = sortedSamples.front().weight * kWeightCutoff;

    std::copy_if(sortedSamples.begin(),
                 sortedSamples.end(),
                 std::back_inserter(filteredSamples),
                 [cutoffWeight](const particle_t& p) {
        return p.weight > cutoffWeight;
    });

    if(filteredSamples.size() < 5)
    {
        filteredSamples.clear();
        std::copy_n(sortedSamples.begin(), 5, std::back_inserter(filteredSamples));
    }

    return filteredSamples.back().weight;
}

} // namespace hssh
} // namespace vulcan
