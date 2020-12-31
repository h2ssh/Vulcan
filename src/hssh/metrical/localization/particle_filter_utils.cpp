/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_filter_utils.cpp
* \author   Collin Johnson
* 
* Definition of functions for sampling from a particle sample set:
* 
*   - draw_samples_from_gaussian
*   - draw_samples_from_cdf
*/

#include "hssh/metrical/localization/particle_filter_utils.h"
#include "core/angle_functions.h"
#include "core/multivariate_gaussian.h"
#include "core/pose_distribution.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{
    
//////////////////// Sampling functions ////////////////////////////////
    
void draw_samples_from_gaussian(const MultivariateGaussian& distribution, std::size_t numSamplesToDraw, std::vector<particle_t>& samples)
{
    assert(distribution.dimensions() == 3);
    
    Vector distributionSample;
    
    for(std::size_t n = 0; n < numSamplesToDraw; ++n)
    {
        distributionSample = distribution.sample();
        
        particle_t sample;
        sample.pose.x     = distributionSample(0);
        sample.pose.y     = distributionSample(1);
        sample.pose.theta = distributionSample(2);
        
        samples.push_back(sample);
    }
}


void draw_samples_from_cdf(const std::vector<particle_t>& cdf, std::size_t numSamplesToDraw, std::vector<particle_t>& samples)
{
    double mInv             = 1.0 / numSamplesToDraw;
    double randomStart      = drand48() * mInv;
    long double cumulativeWeight = cdf[0].weight;
    
    size_t sampleIndex = 0;
    
    for(std::size_t m = 0; m < numSamplesToDraw; ++m)
    {
        double stopCondition = randomStart + (m * mInv);
        
        while(stopCondition > cumulativeWeight)
        {
            ++sampleIndex;
            
            if(sampleIndex == cdf.size())
            {
                --sampleIndex;
                break;
            }
            
            cumulativeWeight += cdf[sampleIndex].weight;
        }
        
        samples.push_back(cdf[sampleIndex]);
    }
}


void draw_samples_around_pose(const pose_t& pose,
                              const MultivariateGaussian& noise,
                              std::size_t numSamplesToDraw,
                              std::vector<particle_t>& samples)
{
    assert(noise.dimensions() == 3);
    decltype(noise.sample()) noiseSample;

    for(std::size_t n = 0; n < numSamplesToDraw; ++n)
    {
        noiseSample = noise.sample();

        particle_t sample;
        sample.pose.x     = pose.x + noiseSample(0);
        sample.pose.y     = pose.y + noiseSample(1);
        sample.pose.theta = wrap_to_pi(pose.theta + noiseSample(2));

        samples.push_back(sample);
    }
}

///////////////////// Distribution functions //////////////////////////////

void normalize_sample_weights(std::vector<particle_t>& particles)
{
    if(particles.empty())
    {
        return;
    }
    
//     double maxLogLikelihood = std::max_element(particles.begin(), particles.end())->weight;
    double sumOfWeights = 0.0;
    
    for(auto& p : particles)
    {
//         p.weight      = std::exp(p.weight - maxLogLikelihood);
        sumOfWeights += p.weight;
    }

    for(auto& p : particles)
    {
        if(p.weight < 0.0)
        {
            std::cout<<"Failed sample weight before:"<<p.weight<<" sum of weights:"<<sumOfWeights<<'\n';
            assert(p.weight >= 0.0);
        }
        
        p.weight /= sumOfWeights;
    }
}


pose_distribution_t calculate_sample_set_distribution(const std::vector<particle_t>& samples)
{
    Vector setMean = calculate_sample_set_mean(samples);
    Matrix setCov  = calculate_sample_set_covariance(samples, setMean);
    
    return pose_distribution_t(MultivariateGaussian(setMean, setCov));
}


Vector calculate_sample_set_mean(const std::vector<particle_t>& samples)
{
    double xMean = 0;
    double yMean = 0;
    double cosThetaMean = 0;
    double sinThetaMean = 0;
    double weightSum = 0.0;
    
    for(int i = samples.size(); --i >= 0;)
    {
        weightSum += samples[i].weight;
        xMean += samples[i].weight * samples[i].pose.x;
        yMean += samples[i].weight * samples[i].pose.y;
        cosThetaMean += samples[i].weight * cos(samples[i].pose.theta);
        sinThetaMean += samples[i].weight * sin(samples[i].pose.theta);
    }
    
    Vector mean(3);
    mean(0) = xMean / weightSum;
    mean(1) = yMean / weightSum;
    mean(2) = atan2(sinThetaMean, cosThetaMean);

    return mean;
}


Matrix calculate_sample_set_covariance(const std::vector<particle_t>& samples,
                                                   const Vector&      mean)
{
    assert(mean.size() == 3);
    assert(samples.size() > 1);
    
    /*
     * Covariance is:
     *
     * [ sigma^2_xx  sigma^2_xy  sigma^2_xtheta
     *               sigma^2_yy  sigma^2_ytheta
     *                           sigma^2_thetatheta]
     *
     * Could do this with matrix cross-products, but the calculation is instead going to be done
     * explicitly. This approach would only come back to bite us if we change to a 6-DOF representation.
     */
    
    const double meanX     = mean(0);
    const double meanY     = mean(1);
    const double meanTheta = mean(2);
    
    double sigmaXX = 0;
    double sigmaYY = 0;
    double sigmaXY = 0;
    double sigmaXTheta = 0;
    double sigmaYTheta = 0;
    double sigmaThetaTheta = 0;
    double sumWeightSquared = 0;
    
    for(int i = samples.size(); --i >= 0;)
    {
        sigmaXX += samples[i].weight * pow(meanX-samples[i].pose.x, 2);
        sigmaYY += samples[i].weight * pow(meanY-samples[i].pose.y, 2);
        
        sigmaXY     += samples[i].weight * (meanX-samples[i].pose.x) * (meanY-samples[i].pose.y);
        sigmaXTheta += samples[i].weight * (meanX-samples[i].pose.x) * angle_diff(meanTheta, samples[i].pose.theta);
        sigmaYTheta += samples[i].weight * (meanY-samples[i].pose.y) * angle_diff(meanTheta, samples[i].pose.theta);
        
        sigmaThetaTheta  += samples[i].weight * pow(angle_diff(meanTheta, samples[i].pose.theta), 2);
        sumWeightSquared += samples[i].weight * samples[i].weight;
    }
    
    Matrix covariance(3, 3);
    
    // Normalize with this value to get the unbiased variance estimate is described here:
    // http://en.wikipedia.org/wiki/Weighted_mean
    double normalizer = (1.0 - sumWeightSquared > 0.0) ? 1.0 / (1.0 - sumWeightSquared) : 1.0;
    
    // Enforce some minimum noise to keep the covariance matrix full-rank
    const double kMinVariance = 1e-8;
    
    sigmaXX = std::max(sigmaXX, kMinVariance);
    sigmaYY = std::max(sigmaYY, kMinVariance);
    sigmaThetaTheta = std::max(sigmaThetaTheta, kMinVariance);
    
    covariance(0, 0) = normalizer * sigmaXX;
    covariance(0, 1) = normalizer * sigmaXY;
    covariance(0, 2) = normalizer * sigmaXTheta;
    covariance(1, 0) = normalizer * sigmaXY;
    covariance(1, 1) = normalizer * sigmaYY;
    covariance(1, 2) = normalizer * sigmaYTheta;
    covariance(2, 0) = normalizer * sigmaXTheta;
    covariance(2, 1) = normalizer * sigmaYTheta;
    covariance(2, 2) = normalizer * sigmaThetaTheta;

    return covariance;
}

} // namespace hssh
} // namespace vulcan
