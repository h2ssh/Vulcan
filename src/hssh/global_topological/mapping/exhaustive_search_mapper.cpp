/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_of_maps_mapper.cpp
* \author   Collin Johnson
*
* Definition of ExhaustiveSearchMapper.
*/

#include <hssh/global_topological/mapping/exhaustive_search_mapper.h>
#include <hssh/global_topological/utils/metric_map_cache.h>
#include <hssh/global_topological/actions.h>
#include <hssh/global_topological/measurements.h>
#include <hssh/global_topological/tree_of_maps.h>
#include <hssh/global_topological/chi.h>
#include <hssh/global_topological/params.h>
#include <hssh/global_topological/mapping/map_optimizer.h>
#include <hssh/global_topological/mapping/localizer.h>
#include <utils/timestamp.h>
#include <iostream>
#include <cassert>

#define DEBUG_PLACE_ENTRY
#define DEBUG_LOOP_CONNECTIONS
#define DEBUG_NEW_PLACE
#define DEBUG_PRUNING
#define DEBUG_LOG_RESULTS

namespace vulcan
{
namespace hssh
{

ExhaustiveSearchMapper::ExhaustiveSearchMapper(const topological_mapper_params_t& params, MetricMapCache& manager)
    : TopologicalMapper(manager)
    , evaluationLog("exhaustive_search_results.log")
{
    optimizer = create_map_optimizer(params.optimizerType, params.optimizerParams);
    localizer = create_topological_localizer(params.localizerType);
}


void ExhaustiveSearchMapper::updateMap(const topology_action_t& action, const topology_measurements_t& measurements)
{
    /*
    * 0) Get active hypotheses
    * 1) For each active hypothesis,
    *    find all frontier places with the same large-scale star
    *    For each frontier place,
    *       find all potential matching segments
    *       For each matching segment,
    *           create a new map hypothesis where the previous place on the path and the matching place are joined by the specified segment
    *           add the new hypothesis to maps to evaluate
    * 2) Eliminate all non-planar graphs (can this be done on-the-fly?)
    * 3) Calculate the likelihood of the new hypotheses
    */

    localizeHypotheses(action);

    // Only do full map updates when a place is entered
    if(!action.haveEntered)
    {
        return;
    }

    if(tree.getHeight() == 0)
    {
        initializeTree(action, measurements);
        return;
    }

    mapsToEvaluate.clear();

    LargeScaleStar actionStar(action.enteredAction.topology, action.enteredAction.entryPath);

    numHypothesesEvaluated = 0;
    numHypothesesExpanded  = 0;

    int64_t startTime = utils::system_time_us();

    pruneInvalidHypotheses(actionStar);
    createNewHypotheses(actionStar, measurements);
    calculateHypothesesProbability(measurements);

    int64_t finishTime = utils::system_time_us();

    const std::set<TopoMapPtr>& leaves = tree.getLeaves();
    uint32_t height = tree.getHeight();

    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        assert((*leafIt)->getDepth() == height-1);
    }

    #ifdef DEBUG_LOG_RESULTS
    evaluationLog<<(tree.getHeight()-1)<<' '<<tree.getLeaves().size()<<' '<<numHypothesesExpanded<<' '<<numHypothesesEvaluated<<' '<<(finishTime - startTime)<<std::endl;
    #endif
}

boost::shared_ptr<TopologicalMapHypothesis> ExhaustiveSearchMapper::getUsableHypothesis(void) const
{
    const std::set<TopoMapPtr>& leaves = tree.getLeaves();

    assert(!leaves.empty());

    return *(leaves.begin());
}


void ExhaustiveSearchMapper::localizeHypotheses(const topology_action_t& action)
{
    const std::set<TopoMapPtr>& leaves = tree.getLeaves();

    localizer->localize(leaves, action);
}


void ExhaustiveSearchMapper::pruneInvalidHypotheses(const LargeScaleStar& actionStar)
{
    /*
    * Each active map will have an associated state indicating the place it should be at according to the
    * loop closures that have been made. Now, when arriving at a new place, check the measured star vs. the
    * required star. If these two are different, then the map is invalid and can be pruned because the assumption
    * for the ExhaustiveSearchMapper is that the correct topology is always measured at a place.
    */

    std::set<TopoMapPtr> leaves(tree.getLeaves());

    GlobalLocation mapState;

    for(auto mapIt = leaves.begin(), mapEnd = leaves.end(); mapIt != mapEnd; ++mapIt)
    {
        mapState = (*mapIt)->getGlobalLocation();

        // If not arrived at a new place, then check the star
        if(mapState.placeId != PLACE_FRONTIER_ID)
        {
            const GlobalPlace& currentPlace = (*mapIt)->getPlace(mapState.placeId);

            if(!currentPlace.getStar().areStarsCompatible(actionStar))
            {
                tree.prune(*mapIt);

                #ifdef DEBUG_PRUNING
                std::cout<<"DEBUG:ExhaustiveSearchMapper:Pruning invalid hypothesis: Stars are not compatible.\n";
                #endif
            }
        }
    }
}


void ExhaustiveSearchMapper::createNewHypotheses(const LargeScaleStar& actionStar, const topology_measurements_t& measurements)
{
    // Create a copy of the active hypotheses, otherwise the addition of new hypotheses will snowball into an infinite loop
    std::set<boost::shared_ptr<TopologicalMapHypothesis>> leaves(tree.getLeaves());

    for(auto mapIt = leaves.begin(), mapEnd = leaves.end(); mapIt != mapEnd; ++mapIt)
    {
        assert(mapIt->get());

        ++numHypothesesExpanded;

        if(atFrontierPlace(*mapIt))
        {
            std::vector<TopoMapPtr> closures = createLoopClosureHypotheses(actionStar, measurements.lambda, measurements.placeId, *mapIt);
            TopoMapPtr              newPlace = createNewPlaceHypothesis(actionStar, measurements.lambda, measurements.placeId, *mapIt);

            mapsToEvaluate.insert(mapsToEvaluate.end(), closures.begin(), closures.end());
            mapsToEvaluate.push_back(newPlace);
        }
        else if(isValidHypothesis(*mapIt, actionStar))
        {
            TopoMapPtr child = moveAlongKnownPath(actionStar, measurements.lambda, measurements.placeId, *mapIt);
            mapsToEvaluate.push_back(child);
        }
        else
        {
            tree.prune(*mapIt);
        }
    }
}


void ExhaustiveSearchMapper::calculateHypothesesProbability(const topology_measurements_t& measurements)
{
    for(auto hypIt = mapsToEvaluate.begin(), hypEnd = mapsToEvaluate.end(); hypIt != hypEnd; ++hypIt)
    {
        Chi hypChi = optimizer->optimizeMap(*(hypIt->get()));
        (*hypIt)->setChi(hypChi);

        ++numHypothesesEvaluated;
    }
}


bool ExhaustiveSearchMapper::isValidHypothesis(const TopoMapPtr& hypothesis, const LargeScaleStar& placeStar)
{
    GlobalLocation mapState = hypothesis->getGlobalLocation();

    // If not arrived at a new place, then check the star
    if(mapState.placeId != PLACE_FRONTIER_ID)
    {
        const GlobalPlace& currentPlace = hypothesis->getPlace(mapState.placeId);

        return currentPlace.getStar().areStarsCompatible(placeStar);
    }

    return true;
}

} // namespace hssh
} // namespace vulcan
