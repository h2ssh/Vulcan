/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_generator_factory_impl.h
* \author   Collin Johnson
* 
* Declaration of subclasses of HypothesisGeneratorFactory:
*   
*   - ExhaustiveGeneratorFactory : creates a factory that generates all children every time
*   - LazyGeneratorFactory : creates a factory that lazily generates children
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_IMPL_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_IMPL_H

#include <hssh/global_topological/mapping/hypothesis_generator_factory.h>
#include <string>

namespace vulcan
{
namespace hssh
{
    
const std::string kExhaustiveGeneratorType("exhaustive");
const std::string kLazyGeneratorType("lazy");

/**
 * ExhaustiveGeneratorFactory creates a HypothesisGenerator that always creates all child hypotheses for its parent
 * state. This generator never changes its probability after being created.
 */
class ExhaustiveGeneratorFactory : public HypothesisGeneratorFactory
{
public:

    /////   HypothesisGeneratorFactory interface   /////
    std::unique_ptr<HypothesisGenerator> createGenerator(const TopologicalState* state, 
                                                         const GlobalLocationDistribution& locations,
                                                         const TopologicalVisit::Ptr& exitVisit,
                                                         const TopologicalVisit::Ptr& entryVisit) override;
};

/**
* LazyGeneratorFactory creates a HypothesisGenerator that lazily creates child hypotheses based on the likelihood the
* robot is there, given the distance it traveled. As new children are completed, the probability of the generator will
* change to reflect the estimate.
*/
class LazyGeneratorFactory : public HypothesisGeneratorFactory
{
public:
    
    /////   HypothesisGeneratorFactory interface   /////
    std::unique_ptr<HypothesisGenerator> createGenerator(const TopologicalState* state, 
                                                         const GlobalLocationDistribution& locations,
                                                         const TopologicalVisit::Ptr& exitVisit,
                                                         const TopologicalVisit::Ptr& entryVisit) override;

};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_FACTORY_IMPL_H
