/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_generator.cpp
 * \author   Collin Johnson
 *
 * Definition of HypothesisGenerator abstract base class.
 */

#include "hssh/global_topological/mapping/hypothesis_generator.h"
#include "hssh/global_topological/mapping/loop_closures.h"
#include "hssh/global_topological/mapping/probability_heuristics.h"
#include "hssh/global_topological/utils/local_to_global.h"
#include <boost/range/iterator_range.hpp>
#include <cassert>

#define DEBUG_LOOP_CLOSURES

namespace vulcan
{
namespace hssh
{

HypothesisGenerator::HypothesisGenerator(const TopologicalState* state,
                                         const GlobalLocationDistribution& locations,
                                         const TopologicalVisit::Ptr& exitVisit,
                                         const TopologicalVisit::Ptr& entryVisit)
: depth_(entryVisit->depth())
, parent_(state)
, locations_(locations)
, exitVisit_(exitVisit)
, entryVisit_(entryVisit)
{
    assert(depth_ >= 0);
}


HypothesisGenerator::~HypothesisGenerator(void)
{
}


TopologicalState HypothesisGenerator::nextState(void)
{
    if (!haveGeneratedChildren_) {
        generateChildren();
        haveGeneratedChildren_ = true;
    }

    TopologicalState next;

    // If not finished yet, snag the last child and remove it
    if (!completed()) {
        assert(!children_.empty());
        next = children_.back().second;
        children_.pop_back();
    }

    return next;
}


bool HypothesisGenerator::completed(void) const
{
    // If there aren't any children, generation is complete!
    return haveGeneratedChildren_ && children_.empty();
}


double HypothesisGenerator::logProbability(void) const
{
    // If no children remain, then there's no probability
    if (haveGeneratedChildren_ && children_.empty()) {
        return -1.0e100;
    } else {
        // Everything needs to be log-likelihood and probability
        // Only have a child probability if the maps have been generated, until then just the parent + heuristic is it
        double childProbability = children_.empty() ? 0.0 : children_.back().first;
        // The prior for the parent doesn't matter, as we just assume it will have the best prior in the future state.
        // The prior can't normally increase
        double prior = std::min(parent_->probability.logPrior, heuristicPrior_);
        return parent_->probability.logLikelihood + heuristicLikelihood_ + prior + childProbability;
    }
}


void HypothesisGenerator::computeProbability(const ProbabilityHeuristics& heuristics)
{
    heuristicPrior_ = heuristics.logPriorHeuristic(depth_);
    heuristicLikelihood_ = heuristics.logLikelihoodHeuristic(depth_);
}


double HypothesisGenerator::computeChildLogLikelihood(const TopologicalState& parent,
                                                      const TopologicalState& child,
                                                      const TopologicalVisit& visit)
{
    // By default all children are equally likely
    return 0.0;
}


void HypothesisGenerator::generateChildren(void)
{
    // Create all children for each possible robot location and find their probabilities
    for (auto& loc : locations_) {
        generateChildrenForLocation(loc);
    }

    // Sort in ascending order
    std::sort(children_.begin(), children_.end(), [](const Child& lhs, const Child& rhs) {
        return lhs.first < rhs.first;
    });
}


void HypothesisGenerator::generateChildrenForLocation(const WeightedGlobalLocation& location)
{
    // If at a new place, then need to add a new place and close all loops
    if (location.location.areaId == kFrontierId) {
        auto newStates = establishPossibleLoopClosures(location.location);
        newStates.push_back(createNewAreaChild(location.location));

        for (auto& state : newStates) {
            double childLogLikelihood = computeChildLogLikelihood(*parent_, state, *exitVisit_);
            children_.emplace_back(location.probability + childLogLikelihood, std::move(state));
        }
    }
    // If at a known place, need to mark the revisited area
    else {
        // When revisiting places, we traversed a path segment, so we need to deal with the updated optimization.
        // Otherwise, the previous optimization is still valid.
        TopologicalState newState;
        newState.id = next_id();
        newState.map = parent_->map->revisitArea(location.location, *entryVisit_);
        newState.location = location.location;
        newState.needsOptimization = is_place_type(location.location.areaType);
        newState.visitDepth = entryVisit_->depth();
        newState.numPlaceVisits = parent_->numPlaceVisits + (is_place_type(location.location.areaType) ? 1 : 0);
        newState.visitEventCount = 0;

        // There is no additional log-likelihood here because we are at a known place
        children_.emplace_back(location.probability, std::move(newState));
    }
}


std::vector<TopologicalState> HypothesisGenerator::establishPossibleLoopClosures(const GlobalLocation& location)
{
    std::vector<TopologicalState> loops;

    // One of these will get created based on the visit type
    GlobalTransition entryTransition;
    GlobalArea newArea(next_id(), location.areaType);
    GlobalPathSegment newSegment;
    GlobalPlace newPlace;

    // Create the appropriate type of place to use for the loop closure checks
    if (is_place_type(location.areaType)) {
        std::tie(newPlace, entryTransition) =
          create_global_place_from_local_place(newArea,
                                               entryVisit_->entryEvent().transitionGateway(),
                                               *entryVisit_->localArea());
    } else if (location.areaType == AreaType::path_segment) {
        std::tie(newSegment, entryTransition) =
          create_global_path_segment_from_local_path_segment(newArea,
                                                             entryVisit_->entryEvent().transitionGateway(),
                                                             *entryVisit_->localArea(),
                                                             false);   // explored state doesn't matter here
    } else {
        std::cerr << "ERROR: HypothesisGenerator: At an unknown type of place: " << location.areaType << '\n';
        assert(location.areaType != AreaType::frontier);
        return loops;
    }

    // Check every frontier to see if it can form a valid loop closure with the newly visited area
    std::cout << "Generating hypotheses. Map has " << parent_->map->sizeFrontiers() << " frontiers.\n";
    for (auto& f : boost::make_iterator_range(parent_->map->beginFrontiers(), parent_->map->endFrontiers())) {
        // Perform an initial sanity check
        bool validLoopClosure = is_possible_loop_closure(location, f, f.visitedArea());

        // If might be a loop closure
        if (validLoopClosure) {
            // Check for specific compatibility with path segment or place, depending on the type
            if (is_place_type(location.areaType) && parent_->map->getPlace(f.visitedArea().id())) {
                validLoopClosure =
                  are_compatible_places(newPlace, entryTransition, *parent_->map->getPlace(f.visitedArea().id()), f);
            } else if (auto path = parent_->map->getPathSegment(f.visitedArea().id())) {
                assert(location.areaType == AreaType::path_segment);
                validLoopClosure = are_compatible_path_segments(newSegment, entryTransition, *path, f);
            }
        }

        // If a valid loop closure was found
        if (validLoopClosure) {
            // Create the corresponding state
            TopologicalState newState;
            newState.id = next_id();
            std::tie(newState.map, newState.location) =
              parent_->map->closeLoop(location, f.visitedArea(), f, *entryVisit_);
            assert(newState.map);
            newState.needsOptimization = is_place_type(location.areaType);   // only optimize upon exiting paths
            newState.visitDepth = entryVisit_->depth();
            newState.numPlaceVisits = parent_->numPlaceVisits + (is_place_type(location.areaType) ? 1 : 0);
            newState.visitEventCount = 0;

            // And add it to the list of possible loop closures
            loops.push_back(std::move(newState));
        }
    }

#ifdef DEBUG_LOOP_CLOSURES
    std::cout << "DEBUG: HypothesisGenerator: Found " << loops.size() << " loop closures for new location amongst "
              << parent_->map->sizeFrontiers() << " frontier areas.\n";
#endif

    return loops;
}


TopologicalState HypothesisGenerator::createNewAreaChild(const GlobalLocation& location)
{
    TopologicalState newState;
    newState.id = next_id();
    std::tie(newState.map, newState.location) = parent_->map->addArea(location, *entryVisit_);
    newState.needsOptimization = false;
    newState.visitDepth = entryVisit_->depth();
    newState.numPlaceVisits = parent_->numPlaceVisits + (is_place_type(location.areaType) ? 1 : 0);
    newState.visitEventCount = 0;

    return newState;
}

}   // namespace hssh
}   // namespace vulcan
