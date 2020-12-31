/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     map_probability.h
 * \author   Collin Johnson
 *
 * Definition of TopoMapProbability.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAP_PROBABILITY_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAP_PROBABILITY_H

#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * TopoMapProbability defines all probability terms pertaining to the probability of a TopologicalMap.
 */
struct TopoMapProbability
{
    std::vector<double> measurementLogLikelihoods;
    double logLikelihood = 0.0;
    double logPrior = 0.0;
    double logPosterior = 0.0;
    double estimatedLogLikelihood = 0.0;
    double estimatedLogPrior = 0.0;

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(measurementLogLikelihoods, logLikelihood, logPrior, logPosterior, estimatedLogLikelihood, estimatedLogPrior);
    }
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAP_PROBABILITY_H
