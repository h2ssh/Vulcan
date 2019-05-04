/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lazy_evaluation_mapper.cpp
* \author   Collin Johnson
*
* Definition of LazyEvaluationMapper.
*/

#include <hssh/global_topological/mapping/lazy_evaluation_mapper.h>
#include <hssh/global_topological/mapping/localizer.h>
#include <hssh/global_topological/mapping/map_optimizer.h>
#include <utils/timestamp.h>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <cstdint>
#include <inttypes.h>

#define DEBUG_HYPOTHESES
#define DEBUG_TOTAL_HYPOTHESES
// #define DEBUG_EVENTS
#define DEBUG_LOG_RESULTS

namespace vulcan
{
namespace hssh
{

LazyEvaluationMapper::LazyEvaluationMapper(const topological_mapper_params_t& params, MetricMapCache& manager)
    : TopologicalMapper(manager)
    , pathEvents(1)
    , probabilityEvaluator(params.lazyEvaluationParams.probabilityParams)
    , params(params.lazyEvaluationParams)
    , evaluationLog("lazy_evaluation_results.log")
    , leafDepthLog("leaf_depths.log")
{
    localizer = create_topological_localizer(params.localizerType);
    optimizer = create_map_optimizer(params.optimizerType, params.optimizerParams);
}


void LazyEvaluationMapper::setCorrectMap(uint32_t id)
{
    setCorrectMap(tree.getMapFromId(id));
}


void LazyEvaluationMapper::setCorrectMap(TopoMapPtr map)
{
    // For setting the correct map, all events in the queue that happened before its depth can, and must, be erased
    // they no longer matter because absolute truth up to the current point has been established
    // After erasing the queue, let the normal TopologicalMapper method do its thing
    if(map && !events.empty())
    {
        uint32_t numEvents = events.size();     // enforce a 32-bit size because size_type is 64-bits on 64-bit Linux
                                                // so std::min() fails in that circumstance
        std::size_t numEventsToErase = std::min(numEvents, map->getDepth());

        events.erase(events.begin(), events.begin()+numEventsToErase);
        pathEvents.erase(pathEvents.begin(), pathEvents.begin()+numEventsToErase);
    }

    mostLikelyHypothesis = map;

    TopologicalMapper::setCorrectMap(map);

    assert(pathEvents.size() == events.size()+1);
    pathEvents.front().clear();     // erase any remaining path events floating around from a previous run
}


void LazyEvaluationMapper::updateMap(const topology_action_t& action, const topology_measurements_t& measurements)
{
    addEventToQueue(action, measurements);

    if(action.haveExited)
    {
        handleExitedEvent(action, measurements);
    }

    if(action.haveEntered)
    {
        handleEnteredEvent(action, measurements);
    }

    if(action.haveReverse)
    {
        handleReverseEvent(action, measurements);
    }

    assert(pathEvents.size() == events.size()+1);
}


void LazyEvaluationMapper::addEventToQueue(const topology_action_t& action, const topology_measurements_t& measurements)
{
    // If entering a new place, then need to add a new event. If exiting, just update the current event
    if(action.haveEntered)
    {
        events.push_back(topology_event_t(action, measurements));
        pathEvents.push_back(reverse_actions_t());
    }

    if(action.haveExited)
    {
        if(!events.empty())
        {
            events.back().exitedAction = action.exitedAction;
        }
        else
        {
            std::cerr<<"WARNING:GlobalTopo: Had an exited place action before an entered action event occurred! Exited place not added to map.\n";
        }
    }

    if(action.haveReverse)
    {
        pathEvents.back().push_back(action.reverseAction);
    }
}


void LazyEvaluationMapper::handleExitedEvent(const topology_action_t& action, const topology_measurements_t& measurements)
{
    // When exiting a place, the action only applies to the complete hypotheses of the tree consistent with all events in the queue. These
    // hypotheses should be localized to include the new exited action so the anything using the output maps doesn't need to
    // wait until a new place is entered before seeing the global location change.

    auto leaves = tree.getCompleteHypotheses();

    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        localizer->localize(*leafIt, action);
    }
}


void LazyEvaluationMapper::handleEnteredEvent(const topology_action_t& action, const topology_measurements_t& measurements)
{
    if(initializeIfNeeded(action, measurements))
    {
        return;
    }

    int64_t startTime = utils::system_time_us();

#ifdef DEBUG_EVENTS
    std::cout<<"DEBUG:LazyEval: Updating.Star:"<<action.enteredAction.enteredAction.topology
    <<" Entry frag:"<<(int)action.enteredAction.enteredAction.entryPath.fragmentId<<':'<<action.enteredAction.enteredAction.entryPath.gateway.getCenter()<<'\n';
#endif

    constructHypothesisQueue();
    numHypothesesExpanded  = 0;
    numHypothesesEvaluated = 0;

    while(!mapQueue.empty() && !haveFinishedUpdate())
    {
        // Pop before expanding, otherwise the wrong value could be popped, as the new hypotheses will be queued during expansion
        TopoMapPtr mapToExpand = mapQueue.top();
        mapQueue.pop();

        expandHypothesis(mapToExpand);

        ++numHypothesesExpanded;

        std::cout<<"num expanded:"<<numHypothesesExpanded<<" most likely:"<<mostLikelyHypothesis->getLogPosterior()<<" depth:"<<mostLikelyHypothesis->getDepth()<<'\n';
    }

    int64_t finishTime = utils::system_time_us();

#ifdef DEBUG_TOTAL_HYPOTHESES
    std::cout<<"DEBUG:LazyEval:Leaves:"<<tree.getLeaves().size()<<" Expanded:"<<numHypothesesExpanded<<" Evaluated:"<<numHypothesesEvaluated<<" Time:"<<((finishTime - startTime)/1000)<<'\n';
#endif

#ifdef DEBUG_LOG_RESULTS
    evaluationLog<<(tree.getHeight()-1)<<' '<<tree.getLeaves().size()<<' '<<numHypothesesExpanded<<' '<<numHypothesesEvaluated<<' '<<(finishTime - startTime)<<std::endl;

    auto leaves = tree.getLeaves();
    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        leafDepthLog<<(*leafIt)->getDepth()<<' ';
    }
    leafDepthLog<<std::endl;
#endif
}


void LazyEvaluationMapper::handleReverseEvent(const topology_action_t& action, const topology_measurements_t& measurements)
{
    auto leaves = tree.getCompleteHypotheses();

    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        localizer->localize(*leafIt, action);
    }
}


bool LazyEvaluationMapper::initializeIfNeeded(const topology_action_t& action, const topology_measurements_t& measurements)
{
    if(tree.getHeight() == 0)
    {
        initializeTree(action, measurements);
        heuristicLikelihood.push_back(0.0);     // no change in log-likelihood at the root
        heuristicPrior.push_back(0.0);
        mostLikelyHypothesis = *(tree.getLeaves().begin());
        return true;
    }

    return false;
}


void LazyEvaluationMapper::constructHypothesisQueue(void)
{
    const std::set<TopoMapPtr>& leaves = tree.getLeaves();

    // Only way to erase the old queue
    mapQueue = std::priority_queue<TopoMapPtr, std::vector<TopoMapPtr>, TopoMapPriority>();

    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        const TopoMapPtr& leaf = *leafIt;

        assert(!leaf->wasPruned());

        hypothesis_probability_t leafProbability = leaf->getProbability();
        calculateEstimatedProbabilities(leaf->getDepth(), leafProbability);
        leaf->setProbability(leafProbability);

        mapQueue.push(leaf);
    }
}


void LazyEvaluationMapper::expandHypothesis(TopoMapPtr& hypothesis)
{
    // Ensure there is a next event, otherwise something is broken. getDepth() == event of creation, + 1 = next event.
    assert(hypothesis.get());
    assert(hypothesis->getDepth() + 1 < events.size());
    assert(!hypothesis->wasPruned());

    const topology_event_t& eventToProcess = enteredPlaceEvent(hypothesis);

#ifdef DEBUG_HYPOTHESES
    std::cout<<"DEBUG:LazyEval:Expanding map. Depth:"<<hypothesis->getDepth()<<" Log:"<<hypothesis->getLogPosterior()<<" Est:"<<hypothesis->getEstimatedLogPosterior()<<' ';
#endif

    localizeHypothesis(hypothesis);

    GlobalLocation location = hypothesis->getGlobalLocation();

    LargeScaleStar actionStar(eventToProcess.enteredAction.enteredAction.topology,
                              eventToProcess.enteredAction.enteredAction.entryPath);

    assert(eventToProcess.enteredAction.enteredAction.entryPath.fragmentId == actionStar.getEntryPathFragment().fragmentId);

    // If at a frontier, then both loop closures and adding a new place are needed
    if(location.placeId == PLACE_FRONTIER_ID)
    {
#ifdef DEBUG_HYPOTHESES
        std::cout<<"Reached new place.\n";
#endif

        addNewPlace(hypothesis, actionStar, eventToProcess.measurements);
        closeLoops (hypothesis, actionStar, eventToProcess.measurements);
    }
    else if(isValidHypothesis(hypothesis, actionStar)) // moved to a known place, so just propagate the map along the tree without branching
    {
#ifdef DEBUG_HYPOTHESES
        std::cout<<"Arrived at known place.\n";
#endif

        movedToKnownPlace(hypothesis, actionStar, eventToProcess);
    }
    else
    {
#ifdef DEBUG_HYPOTHESES
        const LargeScaleStar& expectedStar = hypothesis->getPlace(location.pathState.entryPlaceId).getStar();
        std::cout<<"Pruning. Expected:"<<expectedStar<<':'<<expectedStar.getEntryPathFragment().fragmentId
                 <<" Instead:"<<actionStar<<':'<<actionStar.getEntryPathFragment().fragmentId<<'\n';
#endif

        tree.prune(hypothesis);
    }
}


void LazyEvaluationMapper::localizeHypothesis(TopoMapPtr& hypothesis)
{
    // When a new hypothesis is created, it is localized up to the point of the entered action. If a hypothesis is a leaf consistent
    // with all events in the queue, then its localization is updated when an exited event arrives to make that hypothesis consistent
    // with all actions that have occurred if it is being used for planning. Thus, if a hypothesis is on a path, then the exited event
    // and all path events don't need to be processed. If the hypothesis isn't on a path, then the hypothesis is being expanded from
    // further up the tree so the full localization needs to occur.

    if(!hypothesis->getGlobalLocation().onPath)
    {
        localizer->localize(hypothesis, exitedPlaceEvent(hypothesis).exitedAction);

        for(auto actionIt = reverseActions(hypothesis).begin(), actionEnd = reverseActions(hypothesis).end(); actionIt != actionEnd; ++actionIt)
        {
            localizer->localize(hypothesis, *actionIt);
        }
    }

    localizer->localize(hypothesis, enteredPlaceEvent(hypothesis).enteredAction);
}


void LazyEvaluationMapper::movedToKnownPlace(TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_event_t& event)
{
    TopoMapPtr child = moveAlongKnownPath(placeStar, event.measurements.lambda, event.measurements.placeId, hypothesis);

    processNewMapHypothesis(child, hypothesis, event.measurements);
}


void LazyEvaluationMapper::addNewPlace(TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_measurements_t& measurements)
{
    TopoMapPtr child = createNewPlaceHypothesis(placeStar, measurements.lambda, measurements.placeId, hypothesis);

    processNewMapHypothesis(child, hypothesis, measurements);
}


void LazyEvaluationMapper::closeLoops(TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_measurements_t& measurements)
{
    std::vector<TopoMapPtr> children = createLoopClosureHypotheses(placeStar, measurements.lambda, measurements.placeId, hypothesis);

    std::for_each(children.begin(), children.end(), [&](TopoMapPtr& child) { processNewMapHypothesis(child, hypothesis, measurements); });
}


void LazyEvaluationMapper::processNewMapHypothesis(TopoMapPtr& child, TopoMapPtr& parent, const topology_measurements_t& measurements)
{
    ++numHypothesesEvaluated;

    calculateHypothesisLikelihood(child, parent, measurements);
    updateHeuristic(child, parent);

    // If this hypothesis isn't at the bottom of the tree, then put it back into the queue
    if(!hypothesisContainsAllEvents(child->getDepth()))
    {
        mapQueue.push(child);
    }
    // If the first child to hit the bottom of the tree, then it is automatically the most likely
    else if(mostLikelyHypothesis->getDepth() < child->getDepth())
    {
        mostLikelyHypothesis = child;
    }
    // otherwise, see if it has become the most likely hypothesis
    else if(child->getLogPosterior() > mostLikelyHypothesis->getLogPosterior())
    {
        mostLikelyHypothesis = child;
    }
}


void LazyEvaluationMapper::calculateHypothesisLikelihood(TopoMapPtr& child, TopoMapPtr& parent, const topology_measurements_t& measurements)
{
    if(child->shouldOptimize())
    {
        child->setChi(optimizer->optimizeMap(*(child.get())));
    }

    hypothesis_probability_t probability = probabilityEvaluator.calculateProbability(child, parent, measurements, manager);

    calculateEstimatedProbabilities(child->getDepth(), probability);

    child->setProbability(probability);
}


void LazyEvaluationMapper::calculateEstimatedProbabilities(uint32_t depth, hypothesis_probability_t& probability)
{
    probability.estimatedLogLikelihood = probability.logLikelihood;
    probability.estimatedLogPrior      = probability.logPrior;

    if(params.useHeuristic && !hypothesisContainsAllEvents(depth))
    {
        probability.estimatedLogLikelihood += likelihoodHeurstic(depth);
        probability.estimatedLogPrior       = priorHeuristic(depth, probability.logPrior);
    }

    probability.estimatedLogPosterior = probability.estimatedLogLikelihood + probability.estimatedLogPrior;
}


void LazyEvaluationMapper::updateHeuristic(TopoMapPtr& child, TopoMapPtr& parent)
{
    hypothesis_probability_t childProbability  = child->getProbability();
    hypothesis_probability_t parentProbability = parent->getProbability();

    double logLikelihoodChange = childProbability.logLikelihood - parentProbability.logLikelihood;

    if((child->getDepth() < heuristicLikelihood.size()) && (logLikelihoodChange > heuristicLikelihood[child->getDepth()]))
    {
        heuristicLikelihood[child->getDepth()] = logLikelihoodChange;
    }
    else if(child->getDepth() == heuristicLikelihood.size())
    {
        heuristicLikelihood.push_back(logLikelihoodChange);
    }

    if(child->getDepth() == heuristicPrior.size())
    {
        heuristicPrior.push_back(childProbability.logPrior);
    }

    // There is only a single heuristic prior for the whole tree, as the prior is not recursive
    // If this child is at the bottom of the tree and has the best prior, that becomes the heuristic for ALL nodes
    if(hypothesisContainsAllEvents(child->getDepth()) && (childProbability.logPrior >= heuristicPrior[child->getDepth()]))
    {
        std::fill(heuristicPrior.begin(), heuristicPrior.end(), childProbability.logPrior);
    }

}


double LazyEvaluationMapper::likelihoodHeurstic(uint32_t hypothesisDepth)
{
    assert(hypothesisDepth < heuristicLikelihood.size());
    return std::accumulate(heuristicLikelihood.begin()+hypothesisDepth+1, heuristicLikelihood.end(), 0.0);
}


double LazyEvaluationMapper::priorHeuristic(uint32_t depth, double calculatedPrior)
{
    assert(depth < heuristicPrior.size());
    return (heuristicPrior[depth] < calculatedPrior) ? heuristicPrior[depth] : calculatedPrior;
}


bool LazyEvaluationMapper::isValidHypothesis(TopoMapPtr& hypothesis, const LargeScaleStar& placeStar)
{
    GlobalLocation mapState = hypothesis->getGlobalLocation();

    return (mapState.placeId == PLACE_FRONTIER_ID) || (mapState.placeId >= 0);
}


bool LazyEvaluationMapper::haveFinishedUpdate(void)
{
    return (numHypothesesExpanded >= params.minHypothesesToExpand)                                &&
           (mostLikelyHypothesis->getLogPosterior() > mapQueue.top()->getEstimatedLogPosterior()) &&
           hypothesisContainsAllEvents(mostLikelyHypothesis->getDepth())                          &&
           (numHypothesesEvaluated > 0);
}

} // namespace hssh
} // namespace vulcan
