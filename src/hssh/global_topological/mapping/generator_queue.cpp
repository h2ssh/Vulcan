/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generator_queue.cpp
* \author   Collin Johnson
*
* Definition of GeneratorQueue abstract base class.
*/

#include "hssh/global_topological/mapping/generator_queue.h"
#include "hssh/global_topological/mapping/hypothesis_generator.h"
#include "utils/algorithm_ext.h"
#include <boost/range/adaptor/map.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{

GeneratorQueue::~GeneratorQueue(void)
{
    // For std::unique_ptr
}


bool GeneratorQueue::hasGenerator(const TopologicalState& state) const
{
    return genIds_.find(state.id) != genIds_.end();
}


void GeneratorQueue::addGenerator(std::unique_ptr<HypothesisGenerator> generator)
{
    // Maintain invariant that only active generators go into the queue
    if(!generator->completed())
    {
        HypothesisGenerator* genPtr = generator.get();
        generator->computeProbability(heuristics_);
        queue_.push(genPtr);
        genIds_.insert(generator->parent()->id);
        generators_.insert(std::make_pair(genPtr, std::move(generator)));
    }
    // Otherwise, just ignore the new generator.
}


void GeneratorQueue::setProbabilityHeuristics(const ProbabilityHeuristics& heuristics)
{
    // Recompute the probability for all generators, who rely on the heuristics
    for(auto& g : boost::adaptors::values(generators_))
    {
        g->computeProbability(heuristics);
    }

    // Save the heuristics for any generators added next
    heuristics_ = heuristics;

    // All priorities have potentially changed as a result of changing the heuristics
    rebuildPriorityQueue();
}


void GeneratorQueue::setCompleteDepth(int depth)
{
    depth_ = depth;
    doUpdateCompleteDepth(depth);
}


bool GeneratorQueue::hasNext(void) const
{
    // If there's something in the queue
    if(!queue_.empty())
    {
        // Ask a subclass if the condition for stopping the current update has been met or not
        return shouldGenerateMap(*queue_.top());
    }

    // Queue is empty, so definitely nothing remaining
    return false;
}


std::pair<TopologicalState, const TopologicalState*> GeneratorQueue::nextMap(void)
{
    if(!queue_.empty())
    {
        auto generator = queue_.top();
        queue_.pop();

        auto next = generator->nextState();
        auto parent = generator->parent();

        // If the generator isn't finished yet, toss it back on the queue
        if(!generator->completed())
        {
            queue_.push(generator);
        }
        // Otherwise, the generator completed, so erase it
        // WARNING: Hitting the else statement deletes the generator pointer, so it cannot be used after this statement
        else
        {
            exhausted_.push_back(generator->parent()->id);
            auto numErased = generators_.erase(generator);

            if(numErased != 1)
            {
                std::cerr << "ERROR: GeneratorQueue: Failed to find and erase a completed generator!\n";
            }
        }

        return std::make_pair(next, parent);
    }

    // Otherwise, the queue ended up being empty, so there's no next map
    return std::make_pair(TopologicalState(), nullptr);
}


void GeneratorQueue::addCompleteProbability(const TopoMapProbability& probability)
{
    doUpdateCompleteProbability(probability);
}


void GeneratorQueue::rebuildPriorityQueue(void)
{
    // Clean out the queue
    queue_ = GenQueue();

    // Then push all existing generators back onto it
    for(auto& g : boost::adaptors::values(generators_))
    {
        queue_.push(g.get());
    }
}


bool GeneratorQueue::GenPtrComparator::operator()(const HypothesisGenerator* lhs, const HypothesisGenerator* rhs) const
{
    return lhs->logProbability() < rhs->logProbability();
}

} // namespace hssh
} // namespace vulcan
