/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generator_queue_impl.h
* \author   Collin Johnson
* 
* Declaration of subclasses of GeneratorQueue:
* 
*   - ExhaustiveGeneratorQueue : always processes all children from all generators.
*   - LazyGeneratorQueue : only processes as many children as needed to find the best child map.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_IMPL_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_IMPL_H

#include "hssh/global_topological/mapping/generator_queue.h"
#include <string>

namespace vulcan
{
namespace hssh
{
    
const std::string kExhaustiveGeneratorQueueType("exhaustive");
const std::string kLazyGeneratorQueueType("lazy");
    

/**
* ExhaustiveGeneratorQueue is a simple queue that always evaluates every single generator is contains. Thus,
* shouldGenerateMap() will always return true.
*/
class ExhaustiveGeneratorQueue : public GeneratorQueue
{
public:
    
    ExhaustiveGeneratorQueue(void) = default;
    virtual ~ExhaustiveGeneratorQueue(void);
    
private:

    void doUpdateCompleteDepth(int depth) override;
    void doUpdateCompleteProbability(const TopoMapProbability& probability) override;
    bool shouldGenerateMap(const HypothesisGenerator& generator) const override;
};

/**
* LazyGeneratorQueue is a queue that 
*/
class LazyGeneratorQueue : public GeneratorQueue
{
public:
    
    LazyGeneratorQueue(void) = default;
    virtual ~LazyGeneratorQueue(void);
    
private:
    
    /////   GeneratorQueue interface   /////
    void doUpdateCompleteDepth(int depth) override;
    void doUpdateCompleteProbability(const TopoMapProbability& probability) override;
    bool shouldGenerateMap(const HypothesisGenerator& generator) const override;
    
    int completeDepth_ = 0;
    double bestPosterior_ = -1.0e100;
};

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_IMPL_H
