/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     particle.h
 * \author   Collin Johnson
 *
 * Definition of particle_t.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_SAMPLE_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_SAMPLE_H

#include "core/multivariate_gaussian.h"
#include "core/pose.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
 * particle_t represents a single particle within the particle filter. The particle contains the estimated
 * pose and the particle's weight.
 */
struct particle_t
{
    int id;
    pose_t pose;
    pose_t parent;
    double weight;

    particle_t(float x, float y, float theta, double weight = 1.0) : pose(x, y, theta), weight(weight) { }

    particle_t(void) : id(-1), weight(0) { }
};

/**
 * particle_distribution_t is the distribution represented by the particles in the particle filter.
 * The sampled distribution is stored in samples. A Gaussian approximation of the samples is stored
 * in gaussianApproximation.
 */
struct particle_distribution_t
{
    std::vector<particle_t> samples;
    MultivariateGaussian gaussianApproximation;
};


inline bool operator<(const particle_t& lhs, const particle_t& rhs)
{
    return lhs.weight < rhs.weight;
}

inline bool operator>(const particle_t& lhs, const particle_t& rhs)
{
    return lhs.weight > rhs.weight;
}

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_SAMPLE_H
