/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     mcmc_sampling.cpp
 * \author   Collin Johnson
 *
 * Definition of MCMCSampling.
 */

#include "hssh/local_topological/area_detection/labeling/mcmc_sampling.h"
#include "core/float_comparison.h"
#include "hssh/local_topological/area_detection/labeling/alignment_network_filter.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/boundary_classifier.h"
#include "hssh/local_topological/area_detection/labeling/csp_debug.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_extent.h"
#include "utils/algorithm_ext.h"
#include <algorithm>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <ostream>
#include <random>

#define DEBUG_UPDATES
// #define DEBUG_INITIAL_CONSTRAINTS
// #define DEBUG_FAILED_CONSTRAINTS
// #define DEBUG_CHANGE_SAMPLES
#define DEBUG_AREAS
// #define DEBUG_CONSTRAINTS
// #define DEBUG_AREA_PROB

namespace vulcan
{
namespace hssh
{

const int kChangeLabel = 0;
const int kChangeMerge = 1;
const int kChangeSplit = 2;
const int kChangeNoOp = 3;
const int kChangeGateway = 4;
const int kChangeHierarchy = 5;

const std::size_t kMaxChangeSize = 500;   // can't merge more than this many areas in a single update


///////////////////// MCMCSampling implementation ///////////////////////////////////

MCMCSampling::MCMCSampling(const MCMCSamplingParams& params, const BoundaryClassifier* bndClassifier)
: testSize_(0)
, params_(params)
, bndClassifier_(bndClassifier)
{
    srand48(time(0));

    std::cout << "Initialized MCMC Sampling with:\n"
              << "\tMax iterations:         " << params_.maxIterations << '\n'
              << "\tSamples per iteration:  " << params_.samplesPerIteration << '\n'
              << "\tConstraint log-prob:    " << params_.failingConstraintLogProb << '\n'
              << "\tRepeat config log-prob: " << params_.repeatConfigDecreaseLogProb << '\n';

    constraintPenalty = params_.failingConstraintLogProb;
}


MCMCSampling::Id MCMCSampling::addArea(AreaHypothesis* area, HypothesisType type)
{
    OriginalArea newArea;
    newArea.id = original_.size();
    newArea.active = original_.size();
    newArea.isFixed = false;
    newArea.mustBeConnected = true;
    newArea.hypothesis = std::shared_ptr<AreaHypothesis>(area, [](AreaHypothesis*) {
    });
    newArea.distribution = area->getTypeDistribution();
    // All internal probabilities are log-probabilities
    newArea.distribution.path = std::log(newArea.distribution.path);
    newArea.distribution.destination = std::log(newArea.distribution.destination);
    newArea.distribution.decision = std::log(newArea.distribution.decision);

    if (type == HypothesisType::kArea) {
        newArea.possibleTypes.reserve(3);
        newArea.possibleTypes.push_back(HypothesisType::kDest);
        newArea.possibleTypes.push_back(HypothesisType::kDecision);
        newArea.possibleTypes.push_back(HypothesisType::kPath);
    } else {
        newArea.possibleTypes.push_back(type);
    }

    original_.push_back(newArea);
    active_.emplace_back(newArea, area->getType());
    hypToOrig_[area] = newArea.id;

    totalArea_ += area->extent().area();

    return newArea.active;
}


MCMCSampling::Id MCMCSampling::addFixedArea(AreaHypothesis* area, HypothesisType type, bool isConnected)
{
    OriginalArea newArea;
    newArea.id = original_.size();
    newArea.active = original_.size();
    newArea.isFixed = true;
    newArea.mustBeConnected = isConnected;
    newArea.hypothesis = std::shared_ptr<AreaHypothesis>(area, [](AreaHypothesis*) {
    });
    newArea.distribution = area->getTypeDistribution();
    // All internal probabilities are log-probabilities
    newArea.distribution.path = std::log(newArea.distribution.path);
    newArea.distribution.destination = std::log(newArea.distribution.destination);
    newArea.distribution.decision = std::log(newArea.distribution.decision);

    newArea.possibleTypes.push_back(type);

    original_.push_back(newArea);
    active_.emplace_back(newArea, type);

    // NOTE: Don't add fixed areas to the total area because their probabilities don't factor into the model

    return newArea.active;
}


void MCMCSampling::addConstraint(const AlignmentConstraint& constraint)
{
    OriginalArea& insideArea = original_[constraint.insideId()];
    assert(insideArea.active == constraint.insideId());

    insideArea.constraints.push_back(constraint);
    active(constraint.insideId()).constraints.push_back(constraint);

    // The inside area is adjacent to all outside areas in the constraint
    std::copy(constraint.beginAdjacent(), constraint.endAdjacent(), std::back_inserter(insideArea.adjacent));

    // Look at the adjacent gateways to see if they lead to an endpoint or nonendpoint. Used for the valid-path
    // check when creating new hypothesis areas
    for (auto& adj : boost::make_iterator_range(constraint.beginAdjacent(), constraint.endAdjacent())) {
        if (insideArea.hypothesis->isEndGateway(adj.gatewayId)) {
            insideArea.endpoints.push_back(adj.id);
        } else {
            insideArea.nonendpoints.push_back(adj.id);
        }
    }

    // Ensure the constraint and the area type match and can be satisfied
    if (constraint.type() == AlignmentConstraint::fixed) {
        assert(constraint.fixedType() == active(constraint.insideId()).type);
    }

    // More than one gateway can connect two areas, so ensure only a single adjacency relation is saved
    std::sort(insideArea.adjacent.begin(), insideArea.adjacent.end());
    utils::erase_unique(insideArea.adjacent);

    std::sort(insideArea.endpoints.begin(), insideArea.endpoints.end());
    utils::erase_unique(insideArea.endpoints);

    std::sort(insideArea.nonendpoints.begin(), insideArea.nonendpoints.end());
    utils::erase_unique(insideArea.nonendpoints);

    // Add the adjacent areas to the associated active area
    active(constraint.insideId()).adjacent = insideArea.adjacent;

    if (insideArea.endpoints.size() > 2) {
        std::cerr << "ERROR: MCMCSampling: Somehow found more than two endpoints for an area: Id: " << insideArea.id
                  << " Endpoints:\n";
        std::copy(insideArea.endpoints.begin(), insideArea.endpoints.end(), std::ostream_iterator<int>(std::cerr, ","));
        std::cout << '\n';
        assert(insideArea.endpoints.size() <= 2);
    }

#ifdef DEBUG_INITIAL_CONSTRAINTS
    std::cout << "Adding constraint:" << constraint << '\n';
#endif
}


CSPSolution MCMCSampling::solve(bool doInitialMerge, CSPDebugInfo* debug)
{
#ifdef DEBUG_AREAS
    std::cout << "DEBUG:MCMCSampling::solve: Solving network with the following areas: (id,boundary)\n";
    for (auto& area : active_) {
        std::cout << area.id << " : " << area.hypothesis->rectangleBoundary() << '\n';
    }
#endif

    initializeProbabilities();
    constructGraph();

    if (doInitialMerge) {
        mergeAdjacentSameTypeAreas();
    }

    // Toss some extra areas at the end of the active_ to make sure everything will fit in memory
    // Ensures that active_ is never reallocated, so pointers are always safe to use
    active_.reserve(active_.size() + kMaxChangeSize);
    changeCache_.resize(active_.capacity());

    // Finished with the labels.
    return searchForSolution(debug);
}


bool MCMCSampling::isPathEndGateway(int areaId, int32_t gatewayId, int otherAreaId) const
{
    const ActiveArea& area = active(areaId);

    // Must be a path for it to be a path end gateway.
    if (area.type != HypothesisType::kPath) {
        return false;
    }

    // Check if this gateway is one of the endpoints
    if (area.hypothesis->isEndGateway(gatewayId)) {
        return true;
    }

    // Because boundaries aren't updated when various hypotheses are created, we need to check if
    // any of these end nodes are also contained in the neighboring area. If an end node is also
    // a neighbor's node, then that means they are adjacent via an endpoint.
    const ActiveArea& otherArea = active(otherAreaId);

    for (auto& end : area.hypothesis->endNodes()) {
        if (std::find(otherArea.hypothesis->beginNode(), otherArea.hypothesis->endNode(), end)
            != otherArea.hypothesis->endNode()) {
            return true;
        }
    }

    return false;
}


void MCMCSampling::initializeProbabilities(void)
{
    // For each original area, multiply all probabilties by the fraction of the total area that it occupies
    //     for(auto& orig : original_)
    //     {
    //         double areaLogProb = std::log(orig.hypothesis->extent().area() / totalArea_);
    //         orig.distribution.path += areaLogProb;
    //         orig.distribution.destination += areaLogProb;
    //         orig.distribution.decision += areaLogProb;
    //     }
}


void MCMCSampling::constructGraph(void)
{
    for (auto& orig : original_) {
        auto vertex = boost::add_vertex(graph_);
        assert(static_cast<int>(vertex) == orig.id);
    }

    for (auto& orig : original_) {
        for (auto& adj : orig.adjacent) {
            assert(orig.id < static_cast<int>(original_.size()));
            assert(adj.id < static_cast<int>(original_.size()));
            // Only add edges to those areas with a larger id, this keeps duplicate edges from being added
            if (adj.id > orig.id) {
                boost::add_edge(orig.id, adj.id, graph_);
            }
        }
    }
}


void MCMCSampling::mergeAdjacentSameTypeAreas(void)
{
    // Continually merge adjacent areas with the same type until boundaries only exist between
    // areas with different types
    bool madeChange = true;

    IdVec toMerge;
    int numMerges = 0;

    while (madeChange) {
        madeChange = false;

        for (auto id : activeAreaIds()) {
            ActiveArea& toChange = active_[id];

            // Ignore all fixed areas
            if (toChange.isFixed) {
                continue;
            }

            toMerge = toChange.areas;

            for (auto adj : toChange.adjacent) {
                // No fixed areas should be considered for possible changes
                ActiveArea& adjacent = active(adj.id);
                if (adjacent.isFixed) {
                    continue;
                }

                if (adjacent.type == toChange.type) {
                    boost::push_back(toMerge, boost::as_array(adjacent.areas));
                    madeChange = true;
                }
            }

            // If a merge is made, break out of the loop because the activeIds will have changed
            if (madeChange) {
                auto newArea = createMergedArea(toMerge, toChange.type);
                Id mergeId = newArea.id;
                applyMerge(newArea, mergeId);
                ++numMerges;
                break;
            }
        }
    }

    std::cout << "INFO: MCMCSampling: Made " << numMerges << " initial merges.\n";
}


CSPSolution MCMCSampling::searchForSolution(CSPDebugInfo* debug)
{
    int numAttempts = 0;

    appliedChanges_.clear();
    initialConfig_.original = original_;
    initialConfig_.active = active_;

    validateActiveAreas();

    int numFailingLast = 10000000;
    double initialProb = networkProb(false);
    IdVec areasToSample;
    IdVec areasToChange;

    for (; numAttempts < params_.maxIterations; ++numAttempts) {
        assert(original_.size() == sizeActive());   // confirm the graph is still valid

#ifdef DEBUG_UPDATES
        std::cout << "Starting iteration:" << numAttempts << " Prob:" << networkProb(false) << '\n';
#endif

        // Each new iteration clears out the cache from the previous iteration
        mergedHypotheses_.clear();
        mergedCache_.clear();
        dirtyAreaIds_.clear();

        int numFailing = findInconsistentAreas();

        // Break after saving the debugging info so the final solution is displayed when going to the end of the search
        // sequence in the DebugUI
        if (numFailing == 0) {
            break;
        }

        // When the number of failing constraints doesn't decrease, increase the penalty on constraints failing
        // this will keep the algorithm from getting stuck
        if (numFailing >= numFailingLast) {
            constraintPenalty = -std::pow(std::abs(constraintPenalty), 1.05);
            std::cout << "INFO: MCMCSampling: Constraint penalty: " << constraintPenalty << '\n';
        }
        numFailingLast = numFailing;

        auto activeIds = activeAreaIds();
        for (auto id : activeIds) {
            active_[id].logProb = areaProb(active_[id]);
        }

        gibbsSamples_.clear();
        currentConfig_.original = original_;
        currentConfig_.active = active_;

        std::size_t numSamplesToDraw = params_.samplesPerIteration;
        areasToSample = failingIdsCache_;
        // Don't try to change fixed areas
        utils::erase_remove_if(areasToSample, [this](Id id) {
            return active(id).isFixed;
        });

        // If there are no areas to sample, then we've failed.
        if (areasToSample.empty()) {
            break;
        }

        int maxIterations = numSamplesToDraw * 2;
        int numIterations = 0;

        while ((gibbsSamples_.size() < numSamplesToDraw) && (++numIterations <= maxIterations)) {
            int idToChange = sampleAreaToChangeUniform(areasToSample);

            // Consider changing the area and those areas around it
            areasToChange.clear();
            areasToChange.push_back(active(idToChange).id);
            for (auto& adj : active(idToChange).adjacent) {
                // Only consider changes to non-fixed areas
                if (!active(adj.id).isFixed) {
                    areasToChange.push_back(active(adj.id).id);
                }
            }

            std::sort(areasToChange.begin(), areasToChange.end());
            utils::erase_unique(areasToChange);

            auto sample = drawGibbsSample(areasToChange, idToChange);
            setConfiguration(currentConfig_);

            if (!sample.changes.empty()) {
                gibbsSamples_.push_back(std::move(sample));
            }
        }

        std::cout << "Drew " << gibbsSamples_.size() << " samples from " << areasToSample.size() << " areas.\n";

#ifdef DEBUG_CHANGE_SAMPLES
        std::sort(gibbsSamples_.begin(), gibbsSamples_.end());
        std::cout << "Change samples:\n";
        int count = 0;
        for (auto& sample : gibbsSamples_) {
            std::cout << "Sample " << count++ << ": Prob: " << sample.probability << ":\n";
            for (auto& change : sample.changes) {
                printChange(change);
            }
        }
#endif

        int gibbsIdx = sampleGibbsToApply();
        if (gibbsIdx != -1) {
            boost::push_back(appliedChanges_, boost::as_array(gibbsSamples_[gibbsIdx].changes));
            applySample(gibbsSamples_[gibbsIdx]);
        }

#ifdef DEBUG_UPDATES
        findInconsistentAreas();
        std::cout << "Finished iteration:" << numAttempts << " Prob:" << gibbsSamples_[gibbsIdx].probability
                  << " Total changes:" << appliedChanges_.size() << " Num failing:" << constraintCache_.size() << '\n';
#endif
    }


    CSPSolution solution;

    if (isActiveSolutionConsistent()) {
        double samplingProb = networkProb(false);
        std::cout << "INFO:MCMCSampling: Successfully found a consistent solution in " << numAttempts
                  << " iterations. Beginning local search for better areas...\n";

        bool madeChanges = true;
        while (madeChanges) {
            localSearchForBetterAreas();
            madeChanges = localSamplingForBetterAreas() > 0;
        }

        solution = convertActiveToSolution();

        std::cout << "INFO:MCMCSampling: Completed local search: Start:" << initialProb << " Sampling: " << samplingProb
                  << " Search: " << networkProb(false) << '\n';
    } else {
        dirtyAreaIds_.clear();   // ensure that we search for everything
        findInconsistentAreas();
        // If there's a fixed area with a failing constraint, then that's the cause of the failure.
        bool hasFailingFixed = utils::contains_if(failingIdsCache_, [this](Id id) {
            return active(id).isFixed;
        });

        if (hasFailingFixed) {
            std::cout << "INFO:MCMCSampling: Failing constraints, including fixed:\n";
            for (auto& c : constraintCache_) {
                std::cout << c << '\n';
            }
            solution = CSPSolution(CSPSolution::fixed_area_failing_constraints);
        }

        else {
            solution = CSPSolution(CSPSolution::too_many_attempts);
        }
    }

    if (debug) {
        saveAreaExtents(debug);

        setConfiguration(initialConfig_);
        for (std::size_t n = 0; n < appliedChanges_.size(); ++n) {
            applyChange(appliedChanges_[n]);
            saveIteration(appliedChanges_[n], debug);
        }
    }

    return solution;
}


bool MCMCSampling::isActiveSolutionConsistent(void)
{
    return findInconsistentAreas() == 0;
}


int MCMCSampling::findInconsistentAreas(void)
{
    IdVec activeIds;
    // If there aren't marked dirty areas, then recompute all constraints
    //     if(dirtyAreaIds_.empty())
    //     {
    failingActiveCache_.clear();
    activeIds = activeAreaIds();
    //     }
    // Otherwise, only recompute constraints for areas that are now dirty
    //     else
    //     {
    //         activeIds.resize(dirtyAreaIds_.size());
    //         std::copy(dirtyAreaIds_.begin(), dirtyAreaIds_.end(), activeIds.begin());
    //
    //         // Only clear failing constraint flags for areas that are dirty, as they will be computed later
    //         utils::erase_remove_if(failingActiveCache_, [this](Id id) {
    //             return dirtyAreaIds_.find(active(id).id) != dirtyAreaIds_.end();
    //         });
    //     }

    constraintCache_.clear();
    // Search through every area in the active buffer
    for (auto id : activeIds) {
        // If there's an associated area and at least one constraint fails, then the overall solution isn't consistent
        numFailedActiveConstraints(active(id), &constraintCache_);
    }

    // Check for any areas disconnected from the main graph of decisions and paths
    numFailedGlobalConstraints(&constraintCache_);

    failingIdsCache_.clear();
    for (auto& constraint : constraintCache_) {
        // Don't update failures for non-dirty areas. That results in re-counting failed global constraints
        //         if(dirtyAreaIds_.empty() || dirtyAreaIds_.find(active(constraint.insideId()).id) !=
        //         dirtyAreaIds_.end())
        //         {
        if (constraint.type() != AlignmentConstraint::connectivity) {
            failingActiveCache_.push_back(active(constraint.insideId()).id);
        } else {
            failingIdsCache_.push_back(active(constraint.insideId()).id);
        }
        //         }
    }

    boost::push_back(failingIdsCache_, boost::as_array(failingActiveCache_));
    return failingIdsCache_.size();
}


MCMCSampling::AreaSample MCMCSampling::drawGibbsSample(const IdVec& activeIds, int failingId, bool stopWhenConsistent)
{
    IdVec areasToChange = activeIds;

    ChangeVec possibleChanges;
    ChangeVec appliedChanges;

    bool isFailing = true;
    std::size_t numIterations = 0;
    const std::size_t maxIterations = std::min(std::size_t(25), activeIds.size() * activeIds.size());

    // While there's some area to change and the constraint is failing and we haven't done too much work,
    // keep generating new changes
    while ((isFailing || !stopWhenConsistent) && (numIterations < maxIterations) && !areasToChange.empty()) {
        ++numIterations;

        assert(original_.size() == sizeActive());   // confirm the graph is still valid

        // Compute the probability for all areas to enable the sampling
        findInconsistentAreas();
        auto allActiveIds = activeAreaIds();
        for (auto id : allActiveIds) {
            active_[id].logProb = areaProb(active_[id]);
        }

        int areaId = sampleAreaToChangeUniform(areasToChange);
        possibleChanges.clear();
        if (areaId != -1) {
            areaId = active(areaId).id;
            generatePossibleChanges(active_[areaId], SplitMode::only_borders, possibleChanges);
        }

        int changeIndex = (drand48() < 0.25) ? sampleChangeToApplyUniform(possibleChanges)
                                             : sampleChangeToApplyLogProb(possibleChanges);

        if (changeIndex != -1) {
            auto& changeToApply = possibleChanges[changeIndex];

            applyChange(changeToApply);
            appliedChanges.push_back(changeToApply);

            isFailing = numFailedActiveConstraints(active(failingId)) > 0;

            // Regenerate the areas to change based on which areas are still considered active after applying
            // this change
            areasToChange.clear();
            std::transform(activeIds.begin(), activeIds.end(), std::back_inserter(areasToChange), [this](Id id) {
                return active(id).id;
            });
            std::sort(areasToChange.begin(), areasToChange.end());
            utils::erase_unique(areasToChange);
        }
        // If no changes were found for the area, don't consider it on the next sampling run
        else {
            utils::erase_remove(areasToChange, areaId);
        }
    }

    // NOTE: Always use all the changes because selecting on certain changes will result in getting stuck in a
    // local minimum because all changes that would kick you out of it are discarded.
    AreaSample sample;
    sample.config.original = original_;
    sample.config.active = active_;
    sample.changes = appliedChanges;
    // If there are no changes, then it's just the network prob. Otherwise, the result of applying the changes.
    sample.probability = appliedChanges.empty() ? networkProb() : appliedChanges.back().logProb;
    return sample;
}


int MCMCSampling::generatePossibleChanges(ActiveArea toChange, SplitMode splitMode, ChangeVec& changes)
{
    // No changes are possible if the area contains a fixed constraint
    for (auto& origId : toChange.areas) {
        if (original_[origId].isFixed) {
            // Error if this area has ever been merged!
            assert(toChange.areas.size() == 1);
            return 0;
        }
    }

    // If this area has cached changes, then use them
    if (!changeCache_[toChange.id].empty()) {
        // For each of the changes in the cache, recalculate the strain as the network is slightly different
        // than before
        for (auto& change : changeCache_[toChange.id]) {
            change.logProb = calculateChangeProb(change);
            changes.push_back(change);
        }

        return changeCache_[toChange.id].size();
        ;
    }

    // Otherwise need to do the full search
    std::vector<ActiveArea*> toMerge;
    std::vector<ActiveArea*> adjacentMerge(2);
    adjacentMerge[0] = &toChange;

    // One possible change is to split areas off that were merged from the edge of the area
    // Only try splitting if there's actually multiple areas that could be split off
    if (toChange.areas.size() > 1) {
        for (auto origId : toChange.areas) {
            for (auto adj : original_[origId].adjacent) {
                // If an original area is adjacent to an adjacent area to the active area, then it must be on the
                // outside of the area and can possibly be split off, which would then let it be merged into some other
                // area. Or if always splitting, then split on this origId
                if (utils::contains(toChange.adjacent, adj) || (splitMode == SplitMode::all_areas)) {
                    addSplitChange(toChange, origId, changes);
                    break;
                }
            }
        }
    }

    // Gateway splits will occur between areas that are both in the area to be changed.
    // When finding these, always make the pair have the lower id first, otherwise ignored it.
    // This way duplicates will never appear.
    std::pair<Id, Id> gatewaySplit;
    // One possible change is to split areas off that were merged from the edge of the area
    for (auto origId : toChange.areas) {
        gatewaySplit.first = origId;
        // If an original area is adjacent to an adjacent area to the active area, then it must be on the outside
        // of the area and can possibly be split off, which would then let it be merged into some other area.
        for (auto adj : original_[origId].adjacent) {
            if ((adj.id > origId) && utils::contains(toChange.areas, adj.id)) {
                gatewaySplit.second = adj.id;
                addGatewaySplitChange(toChange, gatewaySplit, changes);
            }
        }
    }

    // Store the initial size of the change vec so number of created changes can be known
    std::size_t initialChangeSize = changes.size();

    // Generate both the option of merging all adjacent and of doing a step-by-step merge
    for (HypothesisType type : toChange.possibleTypes) {
        toMerge.clear();

        for (auto adj : toChange.adjacent) {
            // No fixed areas should be considered for possible changes
            if (original_[adj.id].isFixed) {
                continue;
            }

            ActiveArea& adjacent = active(adj.id);
            if (adjacent.type == type) {
                toMerge.push_back(&adjacent);
                adjacentMerge[1] = &adjacent;

                addMergeChange(toChange, adjacentMerge, type, changes);
            }
        }

        if ((toChange.type != type) && toMerge.empty()) {
            addLabelChange(toChange, type, changes);
        }

        if (toMerge.size() > 1)   // the two-area merge is handled above
        {
            toMerge.push_back(&toChange);
            addMergeChange(toChange, toMerge, type, changes);
        }
    }

    // NOTE: Hierarchy change doesn't really do anything, but is costly to compute, so ignoring it
    //     if(splitMode == SplitMode::all_areas)
    //     {
    //         addHierarchyChanges(toChange, changes);
    //     }

    // Create one more, where all neighbors are merged into this area with its original label
    //     toMerge.clear();
    //     for(auto adj : toChange.adjacent)
    //     {
    //         // No fixed areas should be considered for possible changes
    //         if(original_[adj.id].isFixed)
    //         {
    //             continue;
    //         }
    //
    //         ActiveArea& adjacent = active(adj.id);
    //         toMerge.push_back(&adjacent);
    //     }

    if (toMerge.size() > 1)   // the two-area merge is handled above
    {
        toMerge.push_back(&toChange);
        addMergeChange(toChange, toMerge, toChange.type, changes);
    }

    for (auto& c : changes) {
        for (std::size_t n = 0; n < c.newAreas.size(); ++n) {
            for (std::size_t m = 0; m < c.newAreas.size(); ++m) {
                if (n == m) {
                    continue;
                }

                assert(!utils::contains_any(c.newAreas[n].areas, c.newAreas[m].areas));
            }
        }
    }

    return changes.size() - initialChangeSize;
}


int MCMCSampling::sampleAreaToChangeUniform(const IdVec& activeIds)
{
    // Is there something to sample?
    if (activeIds.empty()) {
        return -1;
    }
    // No need to sample if there's only one choice
    if (activeIds.size() == 1) {
        return activeIds.front();
    }

    // Sample uniformly from the distributions
    int index = random() % activeIds.size();
    return activeIds[index];
}


int MCMCSampling::sampleAreaToChangeLogProb(const IdVec& activeIds)
{
    // Is there something to sample?
    if (activeIds.empty()) {
        return -1;
    }
    // No need to sample if there's only one choice
    if (activeIds.size() == 1) {
        return activeIds.front();
    }

    double maxProb = 0.0;
    double totalProb = std::accumulate(activeIds.begin(), activeIds.end(), 0.0, [this, maxProb](double value, Id id) {
        return value + (1.0 - std::exp(active_[id].logProb - maxProb));
    });

    // Randomly select the probability density to use -- multiply by total rather than divide everything by normalizer
    double idToSelect = drand48() * totalProb;
    double probSoFar = 0.0;

    // Select the first new change whose cdf includes the sampled density.
    for (std::size_t n = 0; n < activeIds.size(); ++n) {
        Id areaId = activeIds[n];

        probSoFar += 1.0 - std::exp(active_[areaId].logProb - maxProb);
        if ((probSoFar > idToSelect) && !active_[areaId].isFixed)   // never sample a fixed area, they can't be changed
        {
            return areaId;
        }
    }

    // If we get to here, then maybe a little rounding error kept us from finishing inside the for-loop
    // so just return the final possibility
    return activeIds.back();
}


int MCMCSampling::sampleChangeToApplyUniform(const ChangeVec& changes)
{
    // Is there something to sample?
    if (changes.empty()) {
        return -1;
    }
    // No need to sample if there's only one choice
    if (changes.size() == 1) {
        return 0;
    }

    // Sample uniformly from the distributions
    std::size_t index = random() % changes.size();
    return std::min(index, changes.size() - 1);
}


int MCMCSampling::sampleChangeToApplyLogProb(const ChangeVec& changes)
{
    if (changes.empty()) {
        return -1;
    }

    auto maxChangeIt = std::max_element(changes.begin(), changes.end());
    double maxProb = maxChangeIt->logProb;

    double totalProb = std::accumulate(changes.begin(), changes.end(), 0.0, [maxProb](double value, auto& change) {
        return value + std::exp(change.logProb - maxProb);
    });

    // Randomly select the probability density to use -- multiply by total rather than divide everything by normalizer
    double changeToSelect = drand48() * totalProb;
    double probSoFar = 0.0;

#ifdef DEBUG_CHANGE_SAMPLES
    std::cout << "DEBUG: MCMCSampling: Sampling from these changes:\n";
    for (auto& chg : changes) {
        double adjProb = std::exp(chg.logProb - maxProb);
        std::cout << "Adj prob:" << adjProb << ' ';
        printChange(chg);
    }
#endif   // DEBUG_CHANGE_SAMPLES

    // Select the first new change whose cdf includes the sampled density.
    for (std::size_t n = 0; n < changes.size(); ++n) {
        probSoFar += std::exp(changes[n].logProb - maxProb);

        if (probSoFar > changeToSelect) {
            return n;
        }
    }

    // If we get to here, then we are accepting the final change
    return changes.size() - 1;
}


int MCMCSampling::sampleGibbsToApply(void)
{
    if (gibbsSamples_.empty()) {
        return -1;
    }

    auto maxChangeIt = std::max_element(gibbsSamples_.begin(), gibbsSamples_.end());
    double maxProb = maxChangeIt->probability;

    double totalProb =
      std::accumulate(gibbsSamples_.begin(), gibbsSamples_.end(), 0.0, [maxProb](double value, auto& change) {
          return value + std::exp(change.probability - maxProb);
      });

    // Randomly select the probability density to use -- multiply by total rather than divide everything by normalizer
    double changeToSelect = drand48() * totalProb;
    double probSoFar = 0.0;

    // Select the first new change whose cdf includes the sampled density.
    for (std::size_t n = 0; n < gibbsSamples_.size(); ++n) {
        probSoFar += std::exp(gibbsSamples_[n].probability - maxProb);

        if (probSoFar > changeToSelect) {
            return n;
        }
    }

    // If we get to here, then we are accepting the final change
    return gibbsSamples_.size() - 1;
}


void MCMCSampling::invalidateCachedChanges(const NetworkChange& applied)
{
    // For each area and adjacent area in the new areas, clear out any changes because they will have different
    // neighbors now, so changes aren't the same as they were

    std::vector<Id> invalidIds;

    for (auto& area : applied.newAreas) {
        boost::push_back(invalidIds, boost::as_array(area.areas));
        for (auto& adj : area.adjacent) {
            invalidIds.push_back(adj.id);
        }
    }

    for (std::size_t n = 0; n < changeCache_.size(); ++n) {
        // Clear out if it is one of the changed areas
        if (utils::contains(invalidIds, n)) {
            changeCache_[n].clear();
        }

        // Or clear out if one of the changes contains one of the bad ids
        bool shouldClear = false;
        for (auto& change : changeCache_[n]) {
            for (auto& area : change.newAreas) {
                shouldClear |= utils::contains_if(area.areas, [&invalidIds](Id id) {
                    return utils::contains(invalidIds, id);
                });

                shouldClear |= utils::contains_if(area.adjacent, [&invalidIds](ConstraintAdjacentArea adj) {
                    return utils::contains(invalidIds, adj.id);
                });
            }

            if (shouldClear) {
                changeCache_[n].clear();
                break;
            }
        }
    }
}


int MCMCSampling::localSamplingForBetterAreas(void)
{
    for (auto& c : changeCache_) {
        c.clear();
    }

    IdVec areasToChange;
    int numChanges = 0;
    bool madeChange = true;

    while (madeChange) {
        madeChange = false;

        // Each new iteration clears out the cache from the previous iteration
        // Clean out the previous config info because it doesn't apply to the local search
        iterConfigs_.clear();
        mergedHypotheses_.clear();
        mergedCache_.clear();
        dirtyAreaIds_.clear();

        findInconsistentAreas();
        auto activeIds = activeAreaIds();
        for (auto id : activeIds) {
            active_[id].logProb = areaProb(active_[id]);
        }

        gibbsSamples_.clear();
        currentConfig_.original = original_;
        currentConfig_.active = active_;

        std::size_t numSamplesToDraw = params_.samplesPerIteration;
        int maxIterations = params_.samplesPerIteration * 2;
        int numIterations = 0;

        while ((gibbsSamples_.size() < numSamplesToDraw) && (++numIterations <= maxIterations)) {
            int idToChange = sampleAreaToChangeLogProb(activeIds);

            // Consider changing the area and those areas around it
            areasToChange.clear();
            areasToChange.push_back(active(idToChange).id);
            for (auto& adj : active(idToChange).adjacent) {
                areasToChange.push_back(active(adj.id).id);
            }

            std::sort(areasToChange.begin(), areasToChange.end());
            utils::erase_unique(areasToChange);

            auto sample = drawGibbsSample(areasToChange, idToChange, false);
            setConfiguration(currentConfig_);

            if (!sample.changes.empty()) {
                setSampleToBestConsistent(sample);
                gibbsSamples_.push_back(std::move(sample));
            }
        }

        double curProb = networkProb();

        int gibbsIdx = sampleGibbsToApply();
        if ((gibbsIdx != -1) && (gibbsSamples_[gibbsIdx].probability > curProb + 1e-6)) {
            madeChange = true;
            ++numChanges;

            boost::push_back(appliedChanges_, boost::as_array(gibbsSamples_[gibbsIdx].changes));
            applySample(gibbsSamples_[gibbsIdx]);

            std::cout << "Sampled better network: Before:" << curProb
                      << " After:" << gibbsSamples_[gibbsIdx].probability << " After via network:" << networkProb()
                      << '\n';
        }
    }

    return numChanges;
}


int MCMCSampling::localSearchForBetterAreas(void)
{
    // For each active area, if it has a better assignment that doesn't break any constraints, and yields a higher
    // overall appropriateness, then apply that assignment. Keep going for each area until the graph settles.

    // Throw out all cached changes at the start because the options for the local search are different than for
    // the solution search. Thus the first iteration will be real slow, but the rest should be faster
    for (ChangeVec& changes : changeCache_) {
        changes.clear();
    }

    // Clean out the previous config info because it doesn't apply to the local search
    iterConfigs_.clear();

    ChangeVec allChanges;
    int numChanges = 0;
    bool madeChange = true;

    while (madeChange) {
        dirtyAreaIds_.clear();
        allChanges.clear();

        findInconsistentAreas();
        auto allActiveIds = activeAreaIds();
        for (auto id : allActiveIds) {
            active_[id].logProb = areaProb(active_[id]);
        }

        // Generate changes to all active areas
        for (auto id : allActiveIds) {
            ActiveArea& active = active_[id];

            // Only both splitting destinations along the boundary because creating other internal structures
            // will just result in low-probability solutions thanks to those areas being disconnected from the
            // rest of the graph.
            auto mode = (active.type == HypothesisType::kDest) ? SplitMode::all_areas : SplitMode::all_areas;

            int numGenerated = generatePossibleChanges(active, mode, allChanges);

            // Cache these changes if there weren't previously cached changes
            if (changeCache_[id].empty()) {
                changeCache_[id].insert(changeCache_[id].end(),
                                        allChanges.begin() + (allChanges.size() - numGenerated),
                                        allChanges.end());
            }
        }

        // Apply the best change -- if a better solution exists
        madeChange = applyBestLocalChange(allChanges);

        if (madeChange) {
            ++numChanges;
        }
    }

    return numChanges;
}


bool MCMCSampling::applyBestLocalChange(ChangeVec& changes)
{
    int bestChangeIndex = -1;
    assert(dirtyAreaIds_.empty());
    double maxProb = networkProb(false);

    for (std::size_t n = 0; n < changes.size(); ++n) {
        // If a change causes no constraints to fail and lowers the strain in the graph, then it should be
        // used instead of the current assignment
        if ((changes[n].numFailing == 0)
            && (changes[n].logProb > maxProb + 1e-6))   // && isNewConfiguration(changes[n]))
        {
            bestChangeIndex = n;
            maxProb = changes[n].logProb;
        }
    }

    if (bestChangeIndex >= 0) {
        invalidateCachedChanges(changes[bestChangeIndex]);
        applyChange(changes[bestChangeIndex]);
        appliedChanges_.push_back(changes[bestChangeIndex]);

        printChange(changes[bestChangeIndex]);
    }

    return bestChangeIndex >= 0;
}


CSPSolution MCMCSampling::convertActiveToSolution(void)
{
    std::vector<AreaLabel> labels;

    for (const auto& area : original_) {
        // If the original id matches the active id, then we'll call this the parent of the active area
        if (area.id == area.active) {
            std::vector<AreaHypothesis*> areaHyps;
            std::transform(active(area.id).areas.begin(),
                           active(area.id).areas.end(),
                           std::back_inserter(areaHyps),
                           [this](Id id) {
                               return original_[id].hypothesis.get();
                           });

            HypothesisTypeDistribution dist;
            if (active(area.id).type == HypothesisType::kPath) {
                dist.path = active(area.id).logProb;
                dist.decision = 0;
                dist.destination = 0;
            } else if (active(area.id).type == HypothesisType::kDest) {
                dist.path = 0;
                dist.decision = 0;
                dist.destination = active(area.id).logProb;
            } else if (active(area.id).type == HypothesisType::kDecision) {
                dist.path = 0;
                dist.decision = active(area.id).logProb;
                dist.destination = 0;
            }
            labels.emplace_back(std::move(areaHyps), active(area.id).type, dist);
        }
    }

    return CSPSolution(labels, networkProb());
}


MCMCSampling::IdVec MCMCSampling::activeAreaIds(void) const
{
    IdVec activeIds;
    activeIds.reserve(sizeActive());
    std::transform(original_.begin(), original_.end(), std::back_inserter(activeIds), [](const OriginalArea& a) {
        return a.active;
    });

    std::sort(activeIds.begin(), activeIds.end());
    utils::erase_unique(activeIds);

    return activeIds;
}


bool MCMCSampling::addNoOpChange(ActiveArea& area, ChangeVec& changes)
{
    NetworkChange change;
    change.sourceId = area.id;
    change.changeType = kChangeNoOp;
    change.logProb = calculateChangeProb(change);
    changes.push_back(std::move(change));

    return true;
}


bool MCMCSampling::addLabelChange(ActiveArea& area, HypothesisType newLabel, ChangeVec& changes)
{
    NetworkChange change;
    change.sourceId = area.id;
    change.newAreas.push_back(area);
    change.newAreas.back().type = newLabel;
    change.newLabels.push_back(newLabel);
    change.changeType = kChangeLabel;
    change.logProb = calculateChangeProb(change);

    changes.push_back(std::move(change));

    return true;
}


bool MCMCSampling::addMergeChange(ActiveArea& area,
                                  const std::vector<ActiveArea*>& toMerge,
                                  HypothesisType newLabel,
                                  ChangeVec& changes)
{
    IdVec idsToMerge;
    for (auto m : toMerge) {
        boost::push_back(idsToMerge, boost::as_array(m->areas));
    }

    std::sort(idsToMerge.begin(), idsToMerge.end());
    utils::erase_unique(idsToMerge);

    NetworkChange change;
    change.sourceId = area.id;
    change.changeType = kChangeMerge;
    change.newLabels.push_back(newLabel);
    change.newAreas.push_back(createMergedArea(idsToMerge, newLabel));

    change.logProb = calculateChangeProb(change);
    changes.push_back(std::move(change));

    return true;
}


bool MCMCSampling::addSplitChange(ActiveArea& area, Id splitId, ChangeVec& changes)
{
    NonSplitActiveEdge<Graph> edgeFilter(&area.areas, splitId, &graph_);
    ActiveVertex<Graph> vertexFilter(&area.areas, &graph_);
    boost::filtered_graph<Graph, NonSplitActiveEdge<Graph>, ActiveVertex<Graph>> filtered(graph_,
                                                                                          edgeFilter,
                                                                                          vertexFilter);
    components_.resize(original_.size());
    int numComponents = boost::connected_components(filtered, &components_[0]);

    assert(numComponents
           > 1);   // must be at least two components that are split off, as split isn't connected to any others

    std::vector<IdVec> splitAreas(numComponents);
    // Add each area in the graph to its connected components, which is the post-split area it belongs to
    for (Id area : boost::make_iterator_range(boost::vertices(filtered))) {
        // If the area is the split area, ignore it because we already created a merged
        // area for it.
        if (area != splitId) {
            splitAreas[components_[area]].push_back(area);
        }
    }

    IdVec splitMergeIds;

    for (std::size_t n = 0; n < area.possibleTypes.size(); ++n) {
        auto newLabel = area.possibleTypes[n];

        splitMergeIds.clear();
        splitMergeIds.push_back(splitId);

        for (auto adj : original_[splitId].adjacent) {
            ActiveArea& adjacent = active(adj.id);
            if ((adjacent.id != area.id) && (adjacent.type == newLabel) && !original_[adj.id].isFixed) {
                boost::push_back(splitMergeIds, boost::as_array(adjacent.areas));
            }
        }

        std::sort(splitMergeIds.begin(), splitMergeIds.end());
        utils::erase_unique(splitMergeIds);

        ActiveArea splitMergeArea = createMergedArea(splitMergeIds, newLabel);

        NetworkChange change;
        change.sourceId = area.id;

        for (auto& split : splitAreas) {
            if (!split.empty()) {
                // If the ids contains the split id, then assign the new label.
                change.newLabels.push_back(area.type);
                change.newAreas.push_back(createMergedArea(split, area.type));
            }
        }

        change.newLabels.push_back(newLabel);
        change.newAreas.push_back(splitMergeArea);

        change.changeType = kChangeSplit;
        change.logProb = calculateChangeProb(change);

        changes.push_back(std::move(change));
    }

    // Splits are always allowed
    return true;
}


bool MCMCSampling::addGatewaySplitChange(ActiveArea& area, std::pair<Id, Id> splitIds, ChangeVec& changes)
{
    using GwySplitVertex = ActiveVertex<Graph>;
    using GwySplitEdge = NonSplitGatewayActiveEdge<Graph>;
    using GwySplitGraph = boost::filtered_graph<Graph, GwySplitEdge, GwySplitVertex>;

    GwySplitEdge edgeFilter(&area.areas, splitIds, &graph_);
    GwySplitVertex vertexFilter(&area.areas, &graph_);
    GwySplitGraph filtered(graph_, edgeFilter, vertexFilter);

    components_.resize(original_.size());
    int numComponents = boost::connected_components(filtered, &components_[0]);

    // If two components don't result from the split, then it wasn't a normal boundary, so just ignore it
    if (numComponents != 2) {
        return false;
    }

    std::array<IdVec, 2> splitAreas;
    // Add each area in the graph to its connected components, which is the post-split area it belongs to
    for (Id area : boost::make_iterator_range(boost::vertices(filtered))) {
        splitAreas[components_[area]].push_back(area);
    }

    IdVec splitMergeIds;

    for (std::size_t n = 0; n < area.possibleTypes.size(); ++n) {
        auto newLabel = area.possibleTypes[n];

        for (int idx = 0; idx < 2; ++idx) {
            splitMergeIds = splitAreas[idx];

            for (auto& splitId : splitAreas[idx]) {
                for (auto adj : original_[splitId].adjacent) {
                    ActiveArea& adjacent = active(adj.id);
                    if ((adjacent.id != area.id) && (adjacent.type == newLabel) && !original_[adj.id].isFixed) {
                        boost::push_back(splitMergeIds, boost::as_array(adjacent.areas));
                    }
                }
            }

            std::sort(splitMergeIds.begin(), splitMergeIds.end());
            utils::erase_unique(splitMergeIds);

            ActiveArea splitMergeArea = createMergedArea(splitMergeIds, newLabel);

            NetworkChange change;
            change.sourceId = area.id;
            change.newLabels.push_back(newLabel);
            change.newAreas.push_back(splitMergeArea);

            change.newLabels.push_back(area.type);
            change.newAreas.push_back(createMergedArea(splitAreas[idx == 0 ? 1 : 0], area.type));

            change.changeType = kChangeGateway;
            change.logProb = calculateChangeProb(change);

            changes.push_back(std::move(change));
        }
    }

    NetworkChange change;
    change.sourceId = area.id;
    change.newLabels.push_back(area.type);
    change.newAreas.push_back(createMergedArea(splitAreas[0], area.type));

    change.newLabels.push_back(area.type);
    change.newAreas.push_back(createMergedArea(splitAreas[1], area.type));

    change.changeType = kChangeGateway;
    change.logProb = calculateChangeProb(change);

    changes.push_back(std::move(change));

    // Splits are always allowed
    return true;
}


bool MCMCSampling::addHierarchyChanges(ActiveArea& area, ChangeVec& changes)
{
    // Find the connected components amongst parts of the graph with a failing constraint
    InactiveEdge<Graph> edgeFilter(&area.areas, &graph_);
    InactiveVertex<Graph> vertexFilter(&area.areas, &graph_);
    auto filtered = boost::make_filtered_graph(graph_, edgeFilter, vertexFilter);
    components_.resize(original_.size());
    std::fill(components_.begin(), components_.end(), -1);
    int numComponents = boost::connected_components(filtered, &components_[0]);

    // If there aren't any components, then they can't be any hierarchies created
    if (numComponents == 0) {
        return false;
    }

    std::vector<IdVec> mergedAreas(numComponents);

    // Add each area in the graph to its connected components, which is the post-split area it belongs to
    for (Id area : boost::make_iterator_range(boost::vertices(filtered))) {
        mergedAreas[components_[area]].push_back(area);
    }

    for (auto& merged : mergedAreas) {
        if (merged.size() > 1) {
            ActiveArea mergedArea = createMergedArea(merged, HypothesisType::kDest);

            NetworkChange change;
            change.sourceId = active(merged.front()).id;
            change.newLabels.push_back(HypothesisType::kDest);
            change.newAreas.push_back(mergedArea);

            change.changeType = kChangeHierarchy;
            change.logProb = calculateChangeProb(change);

            changes.push_back(std::move(change));
        }
    }

    return true;
}


double MCMCSampling::calculateChangeProb(NetworkChange& change)
{
    pushChange(change);
    double prob = networkProb();
    change.numFailing = failingIdsCache_.size();
    popChange();

    return prob;
}


bool MCMCSampling::isValidPath(const ActiveArea& path, bool checkAllAreas)
{
    // When doing a relabeling, all areas will need to be checked for the non-endpoint condition. In the case
    // of doing a regular merge, then only the path elements need to be considered

    // Check each area to see if the active area contains any nonendpoint areas. If so, then the area can't be
    // a path.
    pathEndCache_.clear();

    for (auto& id : path.areas) {
        if (!checkAllAreas && (active(id).type != HypothesisType::kPath)) {
            continue;
        }

        for (auto& endId : original_[id].endpoints) {
            bool isAdj = utils::contains_if(path.adjacent, [endId](auto adj) {
                return endId == adj.id;
            });

            if (isAdj && !utils::contains(pathEndCache_, endId)) {
                pathEndCache_.push_back(endId);
            }
        }
    }

    return pathEndCache_.size() < 3;
}


double MCMCSampling::networkProb(bool prediction)
{
    // If this configuration has existed before, then then probability has already been computed
    //     auto configIt = findCurrentConfig();
    double logProbDecrease = 0.0;
    //     if(configIt != iterConfigs_.end())
    //     {
    //         // If not a prediction, don't add the predicted score on
    //         logProbDecrease = params_.repeatConfigDecreaseLogProb; // prediction ?
    //         params_.repeatConfigDecreaseLogProb : 0.0;
    //     }

    // Otherwise, need to find the probability again
    auto allActiveIds = activeAreaIds();
    double totalProb = 0.0;

    // Go through each currently used active area and accumulate the strain
    for (auto& id : allActiveIds) {
        // Only need to recompute if the area is dirty. Otherwise, use the stored value
        //         if(dirtyAreaIds_.empty() || (dirtyAreaIds_.find(id) != dirtyAreaIds_.end()))
        //         {
        double newLogProb = areaProb(active_[id]);

        //             std::cout << "Changing dirty area " << id << " prob from: " << active_[id].logProb << " to "
        //                 << newLogProb << '\n';

        //             active_[id].logProb = newLogProb;
        totalProb += newLogProb;
        //         }
        //         else
        //         {
        //             double logProb = areaProb(active_[id]);
        //             if(!absolute_fuzzy_equal(logProb, active_[id].logProb))
        //             {
        //                 bool isAdj = false;
        //                 for(auto& adj : active_[id].adjacent)
        //                 {
        //                     isAdj |= dirtyAreaIds_.find(active(adj.id).id) != dirtyAreaIds_.end();
        //                 }
        //
        //                 std::cerr << "ERROR: Clean area had different log prob: " << id << " Stored:" <<
        //                 active_[id].logProb
        //                     << " Computed:" << logProb << " Adj to dirty? " << isAdj
        //                     << " Bounds: " << active_[id].hypothesis->rectangleBoundary() << '\n';
        //
        //                 logProb = areaProb(active_[id]);
        //
        //                 assert(absolute_fuzzy_equal(logProb, active_[id].logProb));
        //             }
        //             totalProb += active_[id].logProb;
        //         }
    }

    int numFailing = findInconsistentAreas();
    // For each failing area, there's a decrease in the overall probability as well
    totalProb += constraintPenalty * numFailing;
    totalProb += logProbDecrease;

    return totalProb;
}


double MCMCSampling::areaProb(ActiveArea& area)
{
    // Fixed areas have 100% probability -- we have complete confidence in them
    if (area.isFixed) {
        return 0.0;
    }

    double onProb = 0.0;

    boundaryCache_.clear();
    for (auto& bnd : boost::make_iterator_range(area.hypothesis->beginBoundary(), area.hypothesis->endBoundary())) {
        // NOTE: All active boundary sqrt b/c they will get multiplied by the areaProb on both sides of the boundary
        onProb += bnd->logProb() * 0.5;
        boundaryCache_.insert(bnd);
    }

    double priorProb = 0.0;

    for (auto& adj : area.adjacent) {
        HypothesisType typeA = area.type;
        HypothesisType typeB = active(adj.id).type;

        if (typeA == HypothesisType::kPath) {
            typeA = isPathEndGateway(area.id, adj.gatewayId, adj.id) ? HypothesisType::kPathEndpoint
                                                                     : HypothesisType::kPathDest;
        }

        if (typeB == HypothesisType::kPath) {
            typeB = isPathEndGateway(adj.id, adj.gatewayId, area.id) ? HypothesisType::kPathEndpoint
                                                                     : HypothesisType::kPathDest;
        }

        // NOTE: All active boundary sqrt b/c they will get multiplied by the areaProb on both sides of the boundary
        priorProb += bndClassifier_->classifyBoundaryLog(typeA, typeB, true) * 0.5;
    }

    double areaProb = 0.0;
    double offProb = 0.0;

    for (auto& origId : area.areas) {
        areaProb += original_[origId].distribution.typeAppropriateness(area.type);

        auto& hyp = original_[origId].hypothesis;

        // All inactive boundaries also add strain
        for (auto& bnd : boost::make_iterator_range(hyp->beginBoundary(), hyp->endBoundary())) {
            // NOTE: All boundary sqrt b/c they will get multiplied by the areaProb on both sides of the boundary
            if (boundaryCache_.find(bnd) == boundaryCache_.end()) {
                offProb += bnd->logProbNot() * 0.5;
            }
        }

        // For each adjacent of the original, see if that boundary has been turned off
        for (auto& adj : original_[origId].adjacent) {
            if (utils::contains(area.areas, adj.id)) {
                HypothesisType typeA = area.type;
                HypothesisType typeB = area.type;

                if (typeA == HypothesisType::kPath) {
                    typeA = original_[origId].hypothesis->isEndGateway(adj.gatewayId) ? HypothesisType::kPathEndpoint
                                                                                      : HypothesisType::kPathDest;

                    typeB = original_[adj.id].hypothesis->isEndGateway(adj.gatewayId) ? HypothesisType::kPathEndpoint
                                                                                      : HypothesisType::kPathDest;
                }

                // NOTE: All active boundary sqrt b/c they will get multiplied by the areaProb on both sides of the
                // boundary
                priorProb += bndClassifier_->classifyBoundaryLog(typeA, typeB, false) * 0.5;
            }
        }
    }

    double totalLogProb = areaProb + priorProb + onProb + offProb;

#ifdef DEBUG_AREA_PROB
    std::cout << "Area: " << area.id << " prob: (" << areaProb << ',' << onProb << ',' << offProb << ',' << priorProb
              << ") -> " << totalLogProb << '\n';
#endif


    return totalLogProb;
}


void MCMCSampling::setActiveLabel(ActiveArea& area, HypothesisType label)
{
    // Both the active hypothesis and all of the individual hypotheses need to have their labels changed to ensure
    // the underlying interaction between different hypotheses works. The hypothesis boundaries all point to original
    // AreaHypotheses, thus changing their types works equivalently to keeping track of the boundaries between the
    // active hypotheses, which requires lots more bookkeeping.
    area.type = label;
    area.hypothesis->setType(label);

    for (auto origId : area.areas) {
        original_[origId].hypothesis->setType(label);
    }
}


void MCMCSampling::applyChange(const NetworkChange& change)
{
    // Don't do anything for the no-op
    if (change.changeType != kChangeNoOp) {
        for (auto& newArea : change.newAreas) {
            Id mergeId = newArea.id;
            applyMerge(newArea, mergeId);
        }

        validateActiveAreas();
    }
}


void MCMCSampling::applyMerge(const ActiveArea& mergedArea, Id mergedId)
{
    // After creation, point all the original areas at the new active id
    for (auto origId : mergedArea.areas) {
        // Clear the state from the previous active area for each of the originals
        original_[origId].active = mergedId;
        assert(!original_[origId].isFixed);
    }

    // Assign the active area to the merged state
    active_[mergedId] = mergedArea;
    setActiveLabel(active_[mergedId], active_[mergedId].type);
}


void MCMCSampling::resetActiveToOriginal(Id area, HypothesisType type)
{
    assert(original_[area].hypothesis);

    ActiveArea& toReset = active_[area];
    toReset.id = area;
    toReset.areas.clear();
    toReset.areas.push_back(area);
    toReset.adjacent = original_[area].adjacent;
    toReset.hypothesis = original_[area].hypothesis;
    toReset.constraints = original_[area].constraints;

    setActiveLabel(toReset, type);

    original_[area].active = area;
}


MCMCSampling::ActiveArea MCMCSampling::createMergedArea(const IdVec& areas, HypothesisType type)
{
    assert(!areas.empty());

    // The merged area contains the active area ids of all areas being merged.
    ActiveArea mergedArea;
    mergedArea.type = type;
    mergedArea.areas = areas;
    mergedArea.possibleTypes = original_[areas.front()].possibleTypes;

    for (auto origId : areas) {
        OriginalArea& area = original_[origId];

        // Copy all areas into the set of merged areas
        boost::push_back(mergedArea.adjacent, boost::as_array(area.adjacent));
        // New possible types are the intersection of the original types
        utils::erase_remove_if(mergedArea.possibleTypes, [&area](HypothesisType type) {
            return !utils::contains(area.possibleTypes, type);
        });
    }

    // Must always be at least one valid possible type -- otherwise an attempt to create an incorrect merge was made
    assert(!mergedArea.possibleTypes.empty());

    // Remove any areas from adjacent if they have been merged into the area
    utils::erase_remove_if(mergedArea.adjacent, [&mergedArea](ConstraintAdjacentArea adj) {
        return utils::contains(mergedArea.areas, adj.id);
    });

    std::sort(mergedArea.areas.begin(), mergedArea.areas.end());
    std::sort(mergedArea.adjacent.begin(), mergedArea.adjacent.end());

    utils::erase_unique(mergedArea.areas);
    utils::erase_unique(mergedArea.adjacent);

    for (auto& adj : mergedArea.adjacent) {
        assert(!utils::contains(areas, adj.id));
    }

    // A new hypothesis consisting of all merged hypotheses needs to be created
    mergedArea.hypothesis = createMergedHypothesis(mergedArea.areas);
    mergedArea.constraints = createMergedConstraints(mergedArea);

    mergedArea.id = mergedArea.areas.front();

    return mergedArea;
}


std::shared_ptr<AreaHypothesis> MCMCSampling::createMergedHypothesis(const IdVec& toMerge)
{
    // Search the cache first before creating a new area
    for (auto& cachedArea : mergedCache_) {
        if (cachedArea.first == toMerge) {
            return mergedHypotheses_[cachedArea.second];
        }
    }

    // The cached value wasn't found so a new area needs to be created
    std::vector<AreaHypothesis*> areasToMerge;
    areasToMerge.reserve(toMerge.size());
    for (auto& origId : toMerge) {
        areasToMerge.push_back(original_[origId].hypothesis.get());
    }

    auto area = std::make_shared<AreaHypothesis>(areasToMerge, false);
    mergedHypotheses_.push_back(area);
    // Store the newly created hypothesis for later use
    mergedCache_.push_back(std::make_pair(toMerge, mergedHypotheses_.size() - 1));
    return area;
}


std::vector<AlignmentConstraint> MCMCSampling::createMergedConstraints(const ActiveArea& area)
{
    std::vector<AlignmentConstraint> mergedConstraints;

    constraintCache_.clear();

    for (Id origId : area.areas) {
        std::copy_if(original_[origId].constraints.begin(),
                     original_[origId].constraints.end(),
                     std::back_inserter(constraintCache_),
                     [origId](const AlignmentConstraint& c) {
                         return c.insideId() == origId;
                     });
    }

    addUnalignedConstraints(area, mergedConstraints);
    addAlignedConstraints(area, mergedConstraints);
    addAdjacencyConstraints(area, mergedConstraints);
    addAllNeighborsConstraint(area, mergedConstraints);
    addFixedTypeConstraint(area, mergedConstraints);

#ifdef DEBUG_CONSTRAINTS
    std::cout << "Initial constraints:\n";
    std::copy(constraintCache_.begin(),
              constraintCache_.end(),
              std::ostream_iterator<AlignmentConstraint>(std::cout, "\n"));
    std::cout << "Merged constraints:\n";
    std::copy(mergedConstraints.begin(),
              mergedConstraints.end(),
              std::ostream_iterator<AlignmentConstraint>(std::cout, "\n"));
    std::cout << "Areas:";
    std::copy(area.areas.begin(), area.areas.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << "\nAdjacent:";
    std::copy(area.adjacent.begin(),
              area.adjacent.end(),
              std::ostream_iterator<ConstraintAdjacentArea>(std::cout, " "));
    std::cout << "\n\n";
#endif   // DEBUG_CONSTRAINTS

    return mergedConstraints;
}


void MCMCSampling::addUnalignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    // Find all pairwise-unaligned gateways in the new AreaHypothesis. The unaligned need to be recalculated because
    // the new pattern of gateways will yield different sets of aligned and unaligned. The main problem with making the
    // assumption of   unaligned separately -> unaligned merged is diagonal gateways cutting an area in half. In these
    // situations, unaligned gateways individually can be aligned when considered together. Conversely, these diagonal
    // gateways could create aligned gateways at an L-intersection, but then merging creates a normal unaligned
    // intersection. The downside is the occasional appearance of unaligned gateways in the middle of hallways. For
    // the most part, these are removed during the search

    MCMCSampling::Id insideId = area.areas.front();
    AlignmentConstraint::AdjVec adjacent;

    auto unalignedGateways = area.hypothesis->pairwiseUnalignedGateways();

    for (auto& u : unalignedGateways) {
        adjacent.clear();

        auto firstAdj = adjacentFromGateway(area, u.first->id());
        auto secondAdj = adjacentFromGateway(area, u.second->id());

        adjacent.push_back(firstAdj);
        adjacent.push_back(secondAdj);
        mergedConstraints.push_back(AlignmentConstraint::CreateUnalignedConstraint(adjacent, insideId));
    }
}


void MCMCSampling::addAlignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    // Find all pairwise-aligned gateways in the new AreaHypothesis. The aligned need to be recalculated because
    // the new pattern of gateways will yield different sets of aligned and unaligned. The main problem with making the
    // assumption of aligned separately -> aligned merged is diagonal gateways cutting an area in half. In these
    // situations, aligned gateways individually can be unaligned when considered together. Conversely, these diagonal
    // gateways could create aligned gateways at an L-intersection, but then merging creates a normal aligned
    // intersection.

    MCMCSampling::Id insideId = area.areas.front();
    AlignmentConstraint::AdjVec adjacent;

    auto alignedGateways = area.hypothesis->pairwiseAlignedGateways();

    for (auto& u : alignedGateways) {
        adjacent.clear();

        auto firstAdj = adjacentFromGateway(area, u.first->id());
        auto secondAdj = adjacentFromGateway(area, u.second->id());

        adjacent.push_back(firstAdj);
        adjacent.push_back(secondAdj);
        mergedConstraints.push_back(AlignmentConstraint::CreateAlignedConstraint(adjacent, insideId));
    }
}


void MCMCSampling::addAdjacencyConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    AlignmentConstraint::AdjVec endpoints;

    for (auto& adj : area.adjacent) {
        if (area.hypothesis->isEndGateway(adj.gatewayId)) {
            endpoints.push_back(adj);
        } else {
            // Doesn't matter which is marked as the inside. All areas work because they will point to the same
            // active area when the constraint is checked.
            mergedConstraints.push_back(AlignmentConstraint::CreateAdjacentConstraint(adj, area.areas.front()));
        }
    }

    if (!endpoints.empty()) {
        mergedConstraints.push_back(AlignmentConstraint::CreateEndpointsConstraint(endpoints, area.areas.front()));
    }
}


void MCMCSampling::addAllNeighborsConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    // The adjacent areas are noted by their ConstraintAdjacentArea, thus all-neighbors just needs to create
    // a constraint amongst all of these areas.
    mergedConstraints.push_back(AlignmentConstraint::CreateAllNeighborsConstraint(area.adjacent, area.areas.front()));
}


void MCMCSampling::addFixedTypeConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    for (auto& constraint : constraintCache_) {
        if (constraint.type() == AlignmentConstraint::fixed) {
            mergedConstraints.push_back(
              AlignmentConstraint::CreateFixedConstraint(area.adjacent, area.areas.front(), constraint.fixedType()));
        }
    }
}


ConstraintAdjacentArea MCMCSampling::adjacentFromGateway(const ActiveArea& area, int32_t gatewayId) const
{
    for (auto& adj : area.adjacent) {
        if (adj.gatewayId == gatewayId) {
            return adj;
        }
    }

    return ConstraintAdjacentArea{-1, -1};
}


void MCMCSampling::setSampleToBestConsistent(AreaSample& sample)
{
    // Go through the changes and erase all changes after the highest probability consistent change
    double bestProb = -10000000.0;
    int bestIndex = -1;
    for (std::size_t n = 0; n < sample.changes.size(); ++n) {
        if ((sample.changes[n].numFailing == 0) && (sample.changes[n].logProb > bestProb)) {
            bestProb = sample.changes[n].logProb;
            bestIndex = n;
        }
    }

    sample.probability = bestProb;

    // If there isn't a best index, then erase all the changes
    if (bestIndex < 0) {
        sample.changes.clear();
    }
    // Otherwise erase all changes after the best change
    else {
        sample.changes.erase(sample.changes.begin() + bestIndex + 1, sample.changes.end());
    }
}


void MCMCSampling::applySample(const AreaSample& sample)
{
    setConfiguration(currentConfig_);

    for (auto& change : sample.changes) {
        printChange(change);
        applyChange(change);

        //         auto configIt = findCurrentConfig();
        //
        //         if(configIt != iterConfigs_.end())
        //         {
        //             configIt->logProb += params_.repeatConfigDecreaseLogProb;
        //         }
        //         else
        //         {
        NetworkConfiguration newConfig;
        newConfig.original = original_;
        newConfig.active = active_;
        newConfig.logProb = change.logProb;
        //             iterConfigs_.emplace_back(std::move(newConfig));
        //         }
    }
}


void MCMCSampling::setConfiguration(const NetworkConfiguration& config)
{
    original_ = config.original;
    active_ = config.active;

    for (auto& orig : original_) {
        orig.hypothesis->setType(active(orig.id).type);
        assert(utils::contains(active_[orig.active].areas, orig.id));
    }

    for (auto& act : active_) {
        act.hypothesis->setType(act.type);
    }

    for (auto& id : activeAreaIds()) {
        assert(!active_[id].areas.empty());
        //         assert(absolute_fuzzy_equal(active_[id].logProb, areaProb(active_[id])));
    }
}


void MCMCSampling::pushChange(const NetworkChange& testChange)
{
    // Do nothing for a no-op
    if (testChange.changeType == kChangeNoOp) {
        testSize_ = 0;
        testAssignedActive_.clear();
        return;
    }

    // Set the size of the change being activated
    testSize_ = testChange.newAreas.size();
    testAssignedActive_.clear();
    dirtyAreaIds_.clear();

    assert(testSize_ > 0);
    assert(testSize_ < kMaxChangeSize);

    for (std::size_t n = 0; n < testChange.newAreas.size(); ++n) {
        auto& newArea = testChange.newAreas[n];

        // Stash the original at the end of the active vector
        active_.push_back(active_[newArea.id]);
        // Replace it with the new area
        active_[newArea.id] = newArea;

        // Point all original areas to the new area
        for (auto& origId : newArea.areas) {
            // Save the previously assignments for the areas being tested
            testAssignedActive_.push_back(original_[origId].active);
            // Reassign to point to the new area
            original_[origId].active = newArea.id;
        }

        // Ensure the label is changed to the correct item
        setActiveLabel(active_[newArea.id], testChange.newLabels[n]);

        dirtyAreaIds_.insert(newArea.id);
        for (auto& adj : newArea.adjacent) {
            dirtyAreaIds_.insert(active(adj.id).id);
        }
    }

    //     validateActiveAreas();
}


void MCMCSampling::popChange(void)
{
    // NOthing to pop.
    if ((testSize_ == 0) || (testAssignedActive_.empty())) {
        return;
    }

    // Point all original areas in the active test area back to their assigned area
    std::size_t nextTestAssignedIndex = 0;
    for (std::size_t n = sizeActive(); n < active_.size(); ++n) {
        // Go through all changed original and reassign
        for (auto& origId : active_[active_[n].id].areas) {
            original_[origId].active = testAssignedActive_[nextTestAssignedIndex];
            ++nextTestAssignedIndex;
        }

        // Reassign the stashed to the original active area
        active_[active_[n].id] = active_[n];
        setActiveLabel(active_[active_[n].id], active_[n].type);
    }

    active_.resize(sizeActive());   // throw away the test active areas
    testSize_ = 0;                  // and reset back to no test areas
    dirtyAreaIds_.clear();

    //     validateActiveAreas();
}


MCMCSampling::ConfigVec::iterator MCMCSampling::findCurrentConfig(void)
{
    bool found = false;
    for (auto configIt = iterConfigs_.begin(), configEnd = iterConfigs_.end(); configIt != configEnd; ++configIt) {
        auto& config = *configIt;

        found = true;
        // Original are always same size, so go through in lock-step to compare them
        for (std::size_t n = 0; n < original_.size(); ++n) {
            auto& curActive = active_[original_[n].active];
            auto& confActive = config.active[config.original[n].active];
            // If the original id and active id match, then use this for the active comparison
            if ((curActive.areas.front() == original_[n].id) && (curActive != confActive)) {
                found = false;
                break;
            }
        }

        if (found) {
            return configIt;
        }
    }

    return iterConfigs_.end();
}


int MCMCSampling::numFailedGlobalConstraints(MCMCSampling::ConstraintVec* failed)
{
    disconnectedAreas_.clear();
    findDisconnectedAreas();

    int disconnectedArea = 0;

    for (Id id : disconnectedAreas_) {
        assert(id < static_cast<Id>(active_.size()));
        ActiveArea& area = active_[id];
        disconnectedArea += 1;

        // If failed, then create the connectivity constraint to indicate the badness that is created
        if (failed) {
            assert(!area.areas.empty());
            failed->push_back(AlignmentConstraint::CreateConnectivityConstraint(area.adjacent, area.areas.front()));
        }
    }

    return (disconnectedArea > 0) ? 1 : 0;
}


int MCMCSampling::numFailedActiveConstraints(const ActiveArea& area, ConstraintVec* failed)
{
    int numFailed = 0;

    // Check the AlignmentConstraints for failures
    for (auto& c : area.constraints) {
        if (!c.isSatisfied(*this, area.type)) {
            ++numFailed;

            if (failed) {
                failed->push_back(c);
            }
        }
    }

    // Check to see if the are enough unaligned paths for a decision point. If not, then mark it as failed as well.
    // The unaligned check has to live outside the AlignmentConstraint unfortunately.
    int numOffsetPaths = 0;
    int numUnalignedConstraints = 0;
    for (auto& c : area.constraints) {
        if (c.isUnalignedPathEnds(*this)) {
            ++numOffsetPaths;
        }

        if (c.type() == AlignmentConstraint::unaligned) {
            ++numUnalignedConstraints;
        }
    }

    bool mustBeConnected = false;
    for (auto& origId : area.areas) {
        mustBeConnected |= original_[origId].mustBeConnected;
    }

    // Only a decision point sitting on the boundary of the map is allowed to not have offset paths
    // Can only fail this check if there are actually unaligned constraints to be failed.
    if (((numOffsetPaths == 0) || (numUnalignedConstraints == 0)) && (area.type == HypothesisType::kDecision)
        && mustBeConnected) {
        if (!area.hypothesis->isBoundary()) {
            ++numFailed;

            // If failed, then create the all-neighbors constraint to indicate something needs to change amongst the
            // neighbors
            if (failed) {
                failed->push_back(AlignmentConstraint::CreateAllNeighborsConstraint(area.adjacent, area.areas.front()));
            }
        }
    }

    return numFailed;
}


void MCMCSampling::findDisconnectedAreas(void)
{
    NonDestEdge<Graph, MCMCSampling> edgeFilter = {this, &graph_};
    NonDestVertex<Graph, MCMCSampling> vertexFilter = {this, &graph_};
    auto filtered = boost::make_filtered_graph(graph_, edgeFilter, vertexFilter);
    components_.resize(original_.size());
    std::fill(components_.begin(), components_.end(), -1);
    int numComponents = boost::connected_components(filtered, &components_[0]);

    // If there is more than one component, some subset of the path segments and decision points in the environment are
    // disconnected from the main connected graph.
    if (numComponents > 1) {
        // The areas in the largest component are accepted as not failing the constraint. All other areas not in the
        // largest component are failing the connected constraint
        std::vector<int> componentSizes(numComponents, 0);
        for (const auto& area : boost::make_iterator_range(boost::vertices(filtered))) {
            ++componentSizes[components_[area]];
        }

        int maxComponent =
          std::distance(componentSizes.begin(), std::max_element(componentSizes.begin(), componentSizes.end()));

        // Go through all the original areas. If they aren't a destination, then they are disconnected from the main
        // graph of decisions and paths in the environment.
        for (auto& orig : original_) {
            if ((active(orig.id).type != HypothesisType::kDest)   // only paths and decisions can be disconnected
                && (components_[orig.id] != maxComponent))        // if not part of biggest component, then disconnected
            {
                if (orig.mustBeConnected) {
                    disconnectedAreas_.push_back(active(orig.id).id);
                }

                for (auto& adj : orig.adjacent) {
                    if (original_[adj.id].mustBeConnected)   // only worry if this area must be connected to the graph
                    {
                        disconnectedAreas_.push_back(active(adj.id).id);
                    }
                }
            }
        }
    }

    std::sort(disconnectedAreas_.begin(), disconnectedAreas_.end());
    utils::erase_unique(disconnectedAreas_);
}


void MCMCSampling::validateActiveAreas(void) const
{
    // Confirm the change was sane -- every orig in active area must point to the active id
    auto activeIds = activeAreaIds();
    for (std::size_t n = 0; n < activeIds.size(); ++n) {
        Id id = activeIds[n];

        for (auto orig : active_[id].areas) {
            if (original_[orig].active != id) {
                std::cout << "Original " << orig << " doesn't point to active area " << id << '\n';
                assert(original_[orig].active == id);
            }
        }

        for (std::size_t m = 0; m < activeIds.size(); ++m) {
            if (m != n) {
                Id otherId = activeIds[m];
                if (utils::contains_any(active_[id].areas, active_[otherId].areas)) {
                    std::cout << "Active " << id << " contains area from " << otherId << '\n';
                    const ActiveArea& lhs = active_[id];
                    const ActiveArea& rhs = active_[otherId];
                    for (auto& lhsArea : lhs.areas) {
                        for (auto& rhsArea : rhs.areas) {
                            assert(lhsArea != rhsArea);
                        }
                    }
                    assert(!utils::contains_any(active_[id].areas, active_[otherId].areas));
                }
            }
        }
    }
}


void MCMCSampling::printSample(const AreaSample& sample) const
{
    for (auto& change : sample.changes) {
        printChange(change);
    }
}


void MCMCSampling::printChange(const NetworkChange& change) const
{
#ifdef DEBUG_CHANGE_SAMPLES
    if (change.changeType == kChangeLabel) {
        std::cout << "Applying label: ";
    } else if (change.changeType == kChangeMerge) {
        std::cout << "Applying merge: ";
    } else if (change.changeType == kChangeSplit) {
        std::cout << "Applying split: ";
    } else if (change.changeType == kChangeNoOp) {
        std::cout << "Applying no-op: ";
    } else if (change.changeType == kChangeGateway) {
        std::cout << "Applying gateway: ";
    } else if (change.changeType == kChangeHierarchy) {
        std::cout << "Applying hierarchy: ";
    }

    std::cout << change.sourceId << " Prob:" << change.logProb << " Failing:" << change.numFailing << ' ';

    for (std::size_t n = 0; n < change.newAreas.size(); ++n) {
        std::cout << '(' << change.newAreas[n].id << " :: " << change.newLabels[n] << ") ";
    }
    std::cout << '\n';
#endif   // DEBUG_CHANGE_SAMPLES
}


void MCMCSampling::saveAreaExtents(CSPDebugInfo* debug)
{
    debug->extents.reserve(original_.size());

    for (OriginalArea& orig : original_) {
        debug->extents.push_back(orig.hypothesis->extent());
    }
}


void MCMCSampling::saveIteration(const NetworkChange& change, CSPDebugInfo* debug)
{
    CSPIteration iteration;
    iteration.labels.reserve(original_.size());
    iteration.strains.reserve(original_.size());

    for (OriginalArea& orig : original_) {
        iteration.labels.push_back(active_[orig.active].type);
        iteration.strains.push_back(1.0);
    }

    for (auto& id : activeAreaIds()) {
        ActiveArea& area = active_[id];

        if (area.type == HypothesisType::kPath) {
            iteration.pathEndpoints.push_back(area.hypothesis->endpoints());
        }

        assert(area.hypothesis);
        assert(!area.areas.empty());

        for (auto& bnd : boost::make_iterator_range(area.hypothesis->beginBoundary(), area.hypothesis->endBoundary())) {
            iteration.gateways.push_back(bnd->getGateway().boundary());
        }
    }

    iteration.isFailing.resize(original_.size(), false);

    findInconsistentAreas();
    std::vector<Id> failedIds = failingIdsCache_;
    std::sort(failedIds.begin(), failedIds.end());
    utils::erase_unique(failedIds);

    iteration.failedAreas.reserve(failedIds.size());
    for (Id id : failedIds) {
        ActiveArea& area = active(id);
        if (area.hypothesis) {
            CSPArea failedArea;
            failedArea.boundary = area.hypothesis->rectangleBoundary();
            failedArea.oldType = area.type;
            failedArea.newType = area.type;
            iteration.failedAreas.push_back(std::move(failedArea));
        }

        // Mark the internal original areas as failing
        for (auto origId : area.areas) {
            iteration.isFailing[origId] = true;
        }
    }

#ifdef DEBUG_FAILED_CONSTRAINTS
    std::cout << "DEBUG::Iteration " << debug->iterations.size() << ": Failing constraints:\n";
    for (auto& cst : constraintCache_) {
        std::cout << cst << '\n';
    }
#endif

    if (change.sourceId >= 0) {
        ActiveArea& area = active(change.sourceId);
        if (area.hypothesis) {
            CSPArea changedArea;
            changedArea.boundary = area.hypothesis->rectangleBoundary();
            changedArea.oldType = area.type;
            changedArea.newType = area.type;
            iteration.updatedArea = changedArea;
        }
    }

    debug->iterations.push_back(std::move(iteration));
}

}   // namespace hssh
}   // namespace vulcan
