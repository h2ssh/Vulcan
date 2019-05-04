/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_generator_factory.h
* \author   Collin Johnson
* 
* Declaration of HypothesisGeneratorFactory interface.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_H

#include <hssh/global_topological/utils/visit.h>
#include <memory>

namespace vulcan
{
namespace hssh
{
    
class GlobalLocationDistribution;
class HypothesisGenerator;
struct TopologicalState;

/**
* HypothesisGeneratorFactory is a factory for creating 
*/
class HypothesisGeneratorFactory
{
public:

    /**
    * createGenerator creates a new HypothesisGenerator instance to use for create new child maps.
    * 
    * \param    state           Parent state from which to generate new hypotheses
    * \param    locations       Distribution of possible locations for the robot
    * \param    exitVisit       Visit for the exited area (corresponds to state->visitDepth)
    * \param    entryVisit      Newly entered visit for which new hypotheses are generated 
    *                               (exitVisit->depty() + 1 == entryVisit->depth())
    * \return   A new instance of a subclass of HypothesisGenerator.
    */
    virtual std::unique_ptr<HypothesisGenerator> createGenerator(const TopologicalState* state, 
                                                                 const GlobalLocationDistribution& locations,
                                                                 const TopologicalVisit::Ptr& exitVisit,
                                                                 const TopologicalVisit::Ptr& entryVisit) = 0;
                                                                 
    virtual ~HypothesisGeneratorFactory(void) = default;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_H
