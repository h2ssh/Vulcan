/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_classifier.cpp
 * \author   Collin Johnson
 *
 * Definition of AffordanceLabelingClassifier.
 */

#include "hssh/local_topological/area_detection/affordance_labeling_classifier.h"
#include "hssh/local_topological/area_detection/labeling/area_creator.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "hssh/local_topological/area_detection/labeling/loops_and_trees.h"
#include "hssh/local_topological/area_detection/labeling/parser.h"
#include "hssh/local_topological/area_detection/labeling/small_scale_star_builder.h"
#include "hssh/local_topological/debug_info.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/params.h"
#include "system/debug_communicator.h"
#include "utils/timestamp.h"

namespace vulcan
{
namespace hssh
{

AffordanceLabelingClassifier::AffordanceLabelingClassifier(const area_classifier_params_t& params,
                                                           const std::string& mapName)
: starBuilder(create_small_scale_star_builder(params.starBuilderParams))
, areaBuilder(std::make_shared<AreaBuilder>(starBuilder))
, parser(new AreaParser(starBuilder, params.mcmcParams, mapName))
, debug(new local_area_debug_info_t)
{
}


AffordanceLabelingClassifier::~AffordanceLabelingClassifier(void)
{
}


LabelingError AffordanceLabelingClassifier::classifyAreas(const std::vector<Gateway>& gateways,
                                                          const VoronoiSkeletonGrid& skeleton,
                                                          const VoronoiIsovistField& isovistField,
                                                          const LocalPerceptualMap& lpm)
{
    HypothesisFeatures::ClearCache();

    int64_t startTime = utils::system_time_us();
    auto result = parser->parseGraph(gateways, skeleton, isovistField, *debug);
    int64_t elapsedTime = utils::system_time_us() - startTime;
    totalTime += elapsedTime;
    ++numUpdates;
    std::cout << "INFO: AffordanceLabelingClassifier: Parse time: " << (elapsedTime / 1000)
              << "ms Average:" << (totalTime / numUpdates / 1000) << "ms\n";

    if (result.result == LabelingError::success) {
        assert(!result.proposals.empty());
        areas_.second = result.logProb;
        createAreasFromProposals(result.proposals, skeleton, isovistField, lpm);
        return LabelingError::success;
    }
    // Failed to create proposals, so the graph couldn't be parsed
    else {
        return result.result;
    }
}


void AffordanceLabelingClassifier::processAreaEvents(const LocalAreaEventVec& events)
{
    for (auto& e : events) {
        e->accept(*this);
    }
}


AffordanceLabelingClassifier::Areas AffordanceLabelingClassifier::currentAreas(void) const
{
    return areas_;
}


void AffordanceLabelingClassifier::sendDebug(system::DebugCommunicator& communicator) const
{
    communicator.sendDebug(*debug);
    parser->sendDebug(communicator);
}


void AffordanceLabelingClassifier::visitAreaTransition(const AreaTransitionEvent& event)
{
    // Not guaranteed to have exited an area. The initial transition doesn't have an exit.
    if (event.exitedArea()) {
        parser->handleTransition(event.exitedArea()->id(), event.enteredArea()->id(), event.transitionGateway());
    }
}


void AffordanceLabelingClassifier::visitTurnAround(const TurnAroundEvent& event)
{
    // Nothing needs to be done for turn around events
}


void AffordanceLabelingClassifier::createAreasFromProposals(const std::vector<AreaProposal>& proposals,
                                                            const VoronoiSkeletonGrid& skeleton,
                                                            const VoronoiIsovistField& isovists,
                                                            const LocalPerceptualMap& lpm)
{
    areas_.first.clear();

    std::transform(proposals.begin(), proposals.end(), std::back_inserter(areas_.first), [&](const AreaProposal& p) {
        return areaBuilder->buildArea(p.getId(), p, lpm, skeleton, isovists);
    });
}

}   // namespace hssh
}   // namespace vulcan
