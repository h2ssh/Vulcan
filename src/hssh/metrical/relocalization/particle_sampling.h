/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_sampling.h
* \author   Collin Johnson
*
* Declaration of:
*
*   - generate_uniform_samples_at_position
*   - generate_random_samples_at_position
*/

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_PARTICLE_SAMPLING_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_PARTICLE_SAMPLING_H

#include "core/point.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

struct particle_t;

/**
* generate_uniform_samples_at_position generates samples at a position uniformly spaced from [0, 2pi].
*
* \param[in]    position        Position at which the samples will be located
* \param[in]    numSamples      Number of samples to draw
* \param[in]    weight          Weight to assign the samples
* \param[out]   samples         Places to store the drawn samples
*/
void generate_uniform_samples_at_position(Point<float> position,
                                          int numSamples,
                                          double weight,
                                          std::vector<particle_t>& samples);

/**
* generate_random_samples_at_position generates samples at a position randomly drawn in the range [0, 2pi].
*
* \param[in]    position        Position at which the samples will be located
* \param[in]    numSamples      Number of samples to draw
* \param[in]    weight          Weight to assign the samples
* \param[out]   samples         Places to store the drawn samples
*/
void generate_random_samples_at_position (Point<float> position,
                                          int numSamples,
                                          double weight,
                                          std::vector<particle_t>& samples);

}
}

#endif // HSSH_UTILS_METRICAL_RELOCALIZATION_PARTICLE_SAMPLING_H
