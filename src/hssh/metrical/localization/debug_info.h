/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_DEBUG_INFO_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_DEBUG_INFO_H

#include "core/laser_scan.h"
#include "core/multivariate_gaussian.h"
#include "hssh/metrical/localization/particle.h"
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

struct particle_grid_score_t
{
    int particleId;
    std::vector<double> scores;
    std::vector<Point<float>> endpoints;
};

struct particle_filter_debug_info_t
{
    MultivariateGaussian proposalDistribution;
    std::vector<particle_t> particles;
    std::vector<particle_grid_score_t> particleScores;
    polar_laser_scan_t scan;
};


// Serialization code
template <class Archive>
void serialize(Archive& ar, particle_grid_score_t& score, const unsigned int version)
{
    ar& score.particleId;
    ar& score.scores;
    ar& score.endpoints;
}


template <class Archive>
void serialize(Archive& ar, particle_t& sample, const unsigned int version)
{
    ar& sample.id;
    ar& sample.pose;
    ar& sample.parent;
    ar& sample.weight;
}


template <class Archive>
void serialize(Archive& ar, particle_filter_debug_info_t& info, const unsigned int version)
{
    ar& info.proposalDistribution;
    ar& info.particles;
    ar& info.particleScores;
    ar& info.scan;
}

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_DEBUG_INFO_H
