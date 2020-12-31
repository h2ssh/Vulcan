/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lazy_generator.h
 * \author   Collin Johnson
 *
 * Declaration of LazyGenerator subclass of HypothesisGenerator.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_GENERATOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_GENERATOR_H

#include "hssh/global_topological/mapping/hypothesis_generator.h"

namespace vulcan
{
namespace hssh
{

/**
 * LazyGenerator is a subclass of HypothesisGenerator that lazily creates child hypotheses. The lazy expansion approach
 * considers the Chi layout of the map hypothesis. Using Chi, the estimated distance traveled, along with its
 * uncertainty creates a distribution over likely locations for the robot. Children are ordered according to this
 * distribution. Each time a child is created, the probability of the generator changes.
 */
class LazyGenerator : public HypothesisGenerator
{
public:
    using HypothesisGenerator::HypothesisGenerator;   ///< Inherit the base class constructors -- nothing new is needed

private:
    /////   HypothesisGenerator interface   /////
    double computeChildLogLikelihood(const TopologicalState& parent,
                                     const TopologicalState& child,
                                     const TopologicalVisit& visit) override;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_GENERATOR_H
