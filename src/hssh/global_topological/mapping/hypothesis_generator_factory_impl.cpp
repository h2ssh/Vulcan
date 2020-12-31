/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_generator_factory_impl.cpp
 * \author   Collin Johnson
 *
 * Definition of ExhaustiveGeneratorFactory and LazyGeneratorFactory.
 */

#include "hssh/global_topological/mapping/hypothesis_generator_factory_impl.h"
#include "hssh/global_topological/mapping/hypothesis_generator.h"
#include "hssh/global_topological/mapping/lazy_generator.h"

namespace vulcan
{
namespace hssh
{

std::unique_ptr<HypothesisGenerator>
  ExhaustiveGeneratorFactory::createGenerator(const TopologicalState* state,
                                              const GlobalLocationDistribution& locations,
                                              const TopologicalVisit::Ptr& exitVisit,
                                              const TopologicalVisit::Ptr& entryVisit)
{
    // Create instances of HypothesisGenerator itself
    return std::make_unique<HypothesisGenerator>(state, locations, exitVisit, entryVisit);
}


std::unique_ptr<HypothesisGenerator> LazyGeneratorFactory::createGenerator(const TopologicalState* state,
                                                                           const GlobalLocationDistribution& locations,
                                                                           const TopologicalVisit::Ptr& exitVisit,
                                                                           const TopologicalVisit::Ptr& entryVisit)
{
    // Create instances of LazyGenerator
    return std::make_unique<LazyGenerator>(state, locations, exitVisit, entryVisit);
}

}   // namespace hssh
}   // namespace vulcan
