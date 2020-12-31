/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     topo_slam.cpp
 * \author   Collin Johnson
 *
 * Definition of TopologicalSLAM.
 */

#include "hssh/global_topological/topo_slam.h"
#include "hssh/global_topological/debug/hypothesis_tree.h"
#include "hssh/global_topological/localization/localizer.h"
#include "hssh/global_topological/mapping/generator_queue.h"
#include "hssh/global_topological/mapping/hypothesis_generator.h"
#include "hssh/global_topological/mapping/hypothesis_generator_factory.h"
#include "hssh/global_topological/mapping/map_optimizer.h"
#include "hssh/global_topological/mapping/probability_evaluator.h"
#include "hssh/global_topological/topological_map.h"
#include "system/debug_communicator.h"
#include "utils/serialized_file_io.h"
#include "utils/timestamp.h"
#include <algorithm>
#include <boost/range/iterator_range.hpp>

#define DEBUG_SEARCH

namespace vulcan
{
namespace hssh
{

TopologicalSLAM::TopologicalSLAM(std::unique_ptr<TopologicalLocalizer> localizer,
                                 std::unique_ptr<GeneratorQueue> queue,
                                 std::unique_ptr<HypothesisGeneratorFactory> generatorFactory,
                                 std::unique_ptr<MapOptimizer> optimizer,
                                 std::unique_ptr<HypothesisProbabilityEvaluator> evaluator,
                                 const std::string& resultsFile)
: localizer_(std::move(localizer))
, queue_(std::move(queue))
, generatorFactory_(std::move(generatorFactory))
, optimizer_(std::move(optimizer))
, evaluator_(std::move(evaluator))
, resultsOut_(resultsFile)
{
}


TopologicalSLAM::~TopologicalSLAM(void)
{
    // For std::unique_ptr
}


void TopologicalSLAM::addEvent(const LocalAreaEvent& event, const LocalTopoMap& localMap)
{
    visits_.addEvent(event, localMap);

    CacheEventVisitor cacheVisitor(mapCache_);
    event.accept(cacheVisitor);
}


void TopologicalSLAM::updatePose(const LocalPose& pose)
{
    visits_.addPose(pose);
}


void TopologicalSLAM::estimateTopologicalState(void)
{
    // Can only update if there's at least one event
    if (visits_.numVisits() == 0) {
        return;
    }

    // If the depth isn't at least 0, then the initial map needs to be created
    if (tree_.depth() < 0) {
        initializeRoot();
    } else if (tree_.depth() + 1 < visits_.numVisits()) {
        searchForNextState();
    } else {
        std::cout << "Event didn't grow tree. No work to be done.\n";
    }
}


TopologicalState TopologicalSLAM::bestEstimate(void)
{
    // If there are no complete maps, then return an empty, invalid state
    if (tree_.sizeComplete() == 0) {
        return TopologicalState();
    }

    // Otherwise, find the maximum a posteriori map amongst the complete maps in the tree
    return **std::max_element(tree_.beginComplete(),
                              tree_.endComplete(),
                              [](const TopologicalState* lhs, const TopologicalState* rhs) {
                                  return lhs->probability.logPosterior < rhs->probability.logPosterior;
                              });
}


void TopologicalSLAM::sendDebug(system::DebugCommunicator& communicator)
{
    communicator.sendDebug(tree_.toHypothesisTree(heuristics_));
}


bool TopologicalSLAM::saveTreeOfMaps(const std::string& filename) const
{
    return utils::save_serializable_to_file(filename, tree_);
}


bool TopologicalSLAM::saveVisitSequence(const std::string& filename) const
{
    return utils::save_serializable_to_file(filename, visits_);
}


bool TopologicalSLAM::saveMapCache(const std::string& filename) const
{
    return utils::save_serializable_to_file(filename, mapCache_);
}


void TopologicalSLAM::initializeRoot(void)
{
    auto initialMap = TopologicalMap::CreateTopologicalMap(*visits_.visitAt(0));
    auto initialState = std::make_unique<TopologicalState>();
    initialState->id = next_id();
    initialState->map = initialMap.first;
    initialState->location = initialMap.second;
    initialState->visitDepth = 0;
    initialState->numPlaceVisits = is_place_type(initialMap.second.areaType) ? 1 : 0;

#ifdef DEBUG_SEARCH
    std::cout << "DEBUG::TopoSLAM: Set root state to be " << initialState->id << " at " << initialState->location
              << '\n';
#endif

    tree_.addRootState(std::move(initialState));
}


void TopologicalSLAM::searchForNextState(void)
{
    static int64_t totalTime = 0;

    int64_t startTime = utils::system_time_us();

    // Update the heuristics used by the queue
    queue_->setProbabilityHeuristics(heuristics_);

    // Change the depth for complete nodes in the queue be one beyond the current depth of the tree
    // Update the depth before enqueuing states because we might erase the current best maps and decrease the
    // depth of the tree, which will then get filled out during the update
    int completeDepth = tree_.depth() + 1;
    queue_->setCompleteDepth(completeDepth);

    // Fill the queue with all leaves
    std::vector<Id> invalidLeaves;
    for (auto& leaf : boost::make_iterator_range(tree_.beginLeaves(), tree_.endLeaves())) {
        if (!enqueueState(leaf)) {
            invalidLeaves.push_back(leaf->id);
        }
    }

    // Prune away all leaves that resulted in an invalid state
    for (auto& leaf : invalidLeaves) {
        tree_.pruneState(leaf);
    }

    assert(queue_->hasNext());

    int numExpanded = 0;

    // Until the queue is completed
    while (queue_->hasNext()) {
        // Get the next map
        auto next = queue_->nextMap();
        ++numExpanded;

        // If something went wrong and there's no next state, then ignore the generated result
        if ((next.first.id == kInvalidId) || !next.second) {
            std::cerr << "ERROR: TopologicalSLAM: Popped an invalid state off the queue!\n";
            continue;
        }

        auto nextMap = std::make_unique<TopologicalState>(std::move(next.first));

        // Compute its probability
        if (nextMap->needsOptimization) {
            nextMap->map->setReferenceFrames(optimizer_->optimizeMap(*nextMap->map));
            nextMap->needsOptimization = false;
        }
        nextMap->probability = evaluator_->calculateProbability(*nextMap, mapCache_);

        // Update the heuristics -- only worry about this if there's actually a valid parent
        heuristics_.updatePriorHeuristic(nextMap->visitDepth, nextMap->probability.logPrior);
        heuristics_.updateLikelihoodHeuristic(nextMap->visitDepth,
                                              nextMap->probability.logLikelihood
                                                - next.second->probability.logLikelihood);

        bool isValidState = false;

        // If the map is complete, tell the queue its probability, so the exit condition can be computed correctly.
        if (nextMap->visitDepth == completeDepth) {
            queue_->addCompleteProbability(nextMap->probability);
            isValidState = true;
        }
        // The map isn't complete, so add it to the tree, as it might keep being processed on this iteration
        else {
            isValidState = enqueueState(nextMap.get());
        }

        // Add the child to the tree
        if (isValidState) {
            tree_.addChildState(next.second->id, std::move(nextMap));
        }
    }

    // Once we've finished this update, need to prune any invalid nodes from the tree
    for (auto id : queue_->exhaustedGenerators()) {
        if (tree_.numChildren(id) == 0) {
            int numPruned = tree_.pruneState(id);
            assert(numPruned > 0);
        }
    }

    queue_->clearExhausted();

    int64_t endTime = utils::system_time_us();
    totalTime += endTime - startTime;

    resultsOut_ << tree_.depth() << ' ' << numExpanded << ' ' << tree_.sizeLeaves() << ' ' << tree_.sizeComplete()
                << ' ' << ((endTime - startTime) / 1000) << std::endl;

    std::cout << "Processing time so far: " << (totalTime / 1000) << '\n';
}


bool TopologicalSLAM::enqueueState(TopologicalState* state)
{
    bool isValidState = true;

    if (!queue_->hasGenerator(*state)) {
        // Localize new leaves
        // visits are zero-indexed, so the index corresponds to the depth of a state.
        auto exitVisit = visits_.visitAt(state->visitDepth);
        state->map->incorporateVisit(state->location, *exitVisit);
        auto locations = localizer_->localize(*state, *exitVisit);

        if (locations.isValid()) {
            auto nextVisit = visits_.visitAt(state->visitDepth + 1);
            queue_->addGenerator(generatorFactory_->createGenerator(state, locations, exitVisit, nextVisit));

#ifdef DEBUG_SEARCH
            std::cout << "DEBUG::TopoSLAM: State " << state->id << " Moved from " << state->location << " to \n";
            for (auto& l : locations) {
                std::cout << '\t' << l.location << " with prob " << l.probability << '\n';
            }
#endif
        } else {
            isValidState = false;
        }
    }

    return isValidState;
}

}   // namespace hssh
}   // namespace vulcan
