/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_filter_utils.h
* \author   Collin Johnson
* 
* Declaration of utility functions for dealing with a set of particle_t:
* 
*   Sampling functions:
*   - draw_samples_from_gaussian
*   - draw_samples_from_cdf
* 
*   Distribution function:
*   - normalize_sample_weights
*   - calculate_sample_set_distribution
*   - calculate_sample_set_mean
*   - calculate_sample_set_covariance
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_UTILS_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_UTILS_H

#include <hssh/metrical/localization/particle.h>
#include <core/matrix.h>
#include <core/vector.h>

namespace vulcan
{
class MultivariateGaussian;

namespace hssh
{

//////////////////// Sampling functions ////////////////////////////////

/**
* draw_samples_from_gaussian draws samples from a Gaussian distribution with mean [x, y, theta].
* 
* \param[in]    distribution            Gaussian distribution from which to draw samples
* \param[in]    numSamplesToDraw        Number of samples to be drawn
* \param[out]   samples                 Sample set in which to put the newly drawn samples
*/
void draw_samples_from_gaussian(const MultivariateGaussian& distribution, std::size_t numSamplesToDraw, std::vector<particle_t>& samples);

/**
* draw_samples_from_cdf draws samples from a distribution represented by a weighted set of samples. The samples are drawn using the
* low-variance sampling algorithm in ProbRob Table 4.4, pg. 110.
* 
* \param[in]    cdf                     Distribution represented by samples
* \param[in]    numSamplesToDraw        Number of samples to be drawn
* \param[out]   samples                 Sample set in which to put the newly drawn samples
*/
void draw_samples_from_cdf(const std::vector<particle_t>& cdf, std::size_t numSamplesToDraw, std::vector<particle_t>& samples);

/**
* draw_samples_around_pose draws samples from a zero-mean noise distribution centered on the specified pose. This
* sampling approach can be used to account for unmodeled errors in the motion model.
*
* \param[in]    pose                    Estimated pose
* \param[in]    noise                   Random pose noise
* \param[in]    numSamplesToDraw        Number of samples to be drawn
* \param[out]   samples                 Sample set in which to store the newly drawn samples
*/
void draw_samples_around_pose(const pose_t& pose,
                              const MultivariateGaussian& noise,
                              std::size_t numSamplesToDraw,
                              std::vector<particle_t>& samples);

///////////////////// Distribution functions //////////////////////////////

/**
* normalize_sample_weights normalizes the weights of the provided samples such that they sum to 1.0.
* 
* \param[in,out]    samples         Samples whose weights will be normalized
*/
void normalize_sample_weights(std::vector<particle_t>& samples);

/**
* calculate_sample_set_distribution calculates the weighted distribution for the provided samples.
*/
pose_distribution_t calculate_sample_set_distribution(const std::vector<particle_t>& samples);

/**
* calculate_sample_set_mean calculates the weighted mean of the distribution using the normalized sample weights.
* 
* \param    samples             Samples in the distribution
* \return   The weighted mean of the distribution.
*/
Vector calculate_sample_set_mean(const std::vector<particle_t>& samples);

/**
* calculate_sample_set_covariance calculates the weighted covariance of the distribution using the normalized sample weights.
* 
* \param    samples             Samples in the distribution
* \param    mean                Mean of the distribution
* \return   The weighted covariance of the sample set centered at mean.
*/
Matrix calculate_sample_set_covariance(const std::vector<particle_t>& samples,
                                                   const Vector&                    mean);

}
}


#endif // HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_UTILS_H
