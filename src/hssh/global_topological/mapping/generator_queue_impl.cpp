/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generator_queue_impl.cpp
* \author   Collin Johnson
*
* Definition of GeneratorQueue subclasses:
*
*   - ExhaustiveGeneratorQueue
*   - LazyGeneratorQueue
*/

#include <hssh/global_topological/mapping/generator_queue_impl.h>
#include <hssh/global_topological/mapping/hypothesis_generator.h>
#include <hssh/global_topological/map_probability.h>

namespace vulcan
{
namespace hssh
{

/////   ExhaustiveGeneratorQueue implementation   /////
ExhaustiveGeneratorQueue::~ExhaustiveGeneratorQueue(void) = default;

void ExhaustiveGeneratorQueue::doUpdateCompleteDepth(int depth)
{
    // Don't care about the complete depth
}


void ExhaustiveGeneratorQueue::doUpdateCompleteProbability(const TopoMapProbability& probability)
{
    // Don't care about complete probabilities
}


bool ExhaustiveGeneratorQueue::shouldGenerateMap(const HypothesisGenerator& generator) const
{
    // All generators always generate all maps
    return true;
}

/////   LazyGeneratorQueue implementation   /////
LazyGeneratorQueue::~LazyGeneratorQueue(void) = default;

void LazyGeneratorQueue::doUpdateCompleteDepth(int depth)
{
    bestPosterior_ = -1.0e100;
    completeDepth_ = depth;
}


void LazyGeneratorQueue::doUpdateCompleteProbability(const TopoMapProbability& probability)
{
    // If a new map is completed, reassign the best posterior to amongst the best of the previous best and new map
    if(probability.logPosterior > bestPosterior_)
    {
        std::cout << "Better posterior..." << bestPosterior_ << " -> " << probability.logPosterior << '\n';
        bestPosterior_ = std::max(probability.logPosterior, bestPosterior_);
    }
}


bool LazyGeneratorQueue::shouldGenerateMap(const HypothesisGenerator& generator) const
{
    // A map is generated whenever its log probability is at least as good as the probability of the best posterior
    // seen so far.
    return generator.logProbability() >= bestPosterior_;
}

} // namespace hssh
} // namespace vulcan
