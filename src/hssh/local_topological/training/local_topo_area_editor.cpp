/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_area_editor.cpp
* \author   Collin Johnson
*
* Implementation of LocalTopoAreaEditor.
*/

#include <hssh/local_topological/training/local_topo_area_editor.h>
#include <hssh/local_topological/params.h>
#include <hssh/local_topological/area_detection/area_classifier.h>
#include <hssh/local_topological/area_detection/voronoi_graph_builder.h>
#include <hssh/local_topological/area_detection/gateways/filters.h>
#include <hssh/local_topological/area_detection/gateways/gateway_locator.h>
#include <hssh/local_topological/area_detection/labeling/area_creator.h>
#include <hssh/local_topological/area_detection/labeling/area_graph.h>
#include <hssh/local_topological/area_detection/labeling/area_proposal.h>
#include <hssh/local_topological/area_detection/labeling/boundary.h>
#include <hssh/local_topological/area_detection/labeling/invalid_area.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_graph.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_edges.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_extent.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/decision_point.h>
#include <hssh/local_topological/areas/destination.h>
#include <hssh/local_topological/areas/path_segment.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_features.h>
#include <hssh/local_topological/area_detection/labeling/small_scale_star_builder.h>
#include <hssh/local_metric/lpm.h>
#include <utils/algorithm_ext.h>
#include <utils/serialized_file_io.h>
#include <utils/visibility_graph.h>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{

void add_merge_boundaries(AreaHypothesis::BoundaryIter    boundaryIt,
                          AreaHypothesis::BoundaryIter    boundaryEnd,
                          std::vector<AreaHypothesis*>::const_iterator hypBegin,
                          std::vector<AreaHypothesis*>::const_iterator hypEnd,
                          std::vector<const AreaHypothesisBoundary*>& toMerge);


LocalTopoAreaEditor::LocalTopoAreaEditor(const local_topology_params_t& params)
: voronoiBuilder_(new VoronoiSkeletonBuilder(params.areaParams.skeletonParams, params.areaParams.prunerParams))
, gatewayLocator_(new GatewayLocator(params.areaParams.locatorParams, std::string("")))
, starBuilder_(create_small_scale_star_builder(params.areaParams.classifierParams.starBuilderParams))
, maxIsovistRange_(params.areaParams.maxIsovistRange)
, numIsovistRays_(params.areaParams.numIsovistRays)
, gatewayFeatureRadius_(params.areaParams.locatorParams.generatorParams.classifierParams.featureRadius)
{
}


LocalTopoAreaEditor::~LocalTopoAreaEditor(void)
{
    // For std::unique_ptr
}


void LocalTopoAreaEditor::setGatewayGenerator(std::unique_ptr<GatewayGenerator> generator)
{
    gatewayLocator_ = std::make_unique<GatewayLocator>(std::move(generator));
}


VoronoiSkeletonGrid LocalTopoAreaEditor::buildSkeleton(const LocalPerceptualMap& lpm)
{
    lpm_ = std::make_unique<LocalPerceptualMap>(lpm);

    // For the editor, just assume robot at the origin of the LPM, which means it won't be accounted for
    // when determining exits
    std::vector<Point<float>> pointsOfInterest;   // none since no prior information
    voronoiBuilder_->buildVoronoiSkeleton(lpm, pose_t(lpm.getBottomLeft(), 0.0f), pointsOfInterest);

    // Reset all data stored with the previous representation
    hypotheses_.clear();
    initialHypotheses_.clear();
    mergedHypotheses_.clear();
    areaGraph_.reset();
    hypothesisGraph_.reset();
    initialHypothesisGraph_.reset();
    isovistField_.reset();

    return voronoiBuilder_->getVoronoiSkeleton();
}


std::vector<Gateway> LocalTopoAreaEditor::findGateways(void) const
{
    createIsovistsIfNeeded();
    gatewayLocator_->locateGateways(voronoiBuilder_->getVoronoiSkeleton(), *isovistField_);
    return gatewayLocator_->getGateways();
}


std::vector<Gateway> LocalTopoAreaEditor::findMoreGateways(const std::vector<Gateway>& initialGateways) const
{
    // Create the WeightedGateways for the initial loaded gateways
    std::vector<WeightedGateway> weighted;
    std::transform(initialGateways.begin(),
                   initialGateways.end(),
                   std::back_inserter(weighted),
                   [](auto& gwy) {
        return WeightedGateway{gwy, 1000.0, false};
    });

    // Then find gateways with the locator
    auto located = findGateways();

    std::transform(located.begin(),
                   located.end(),
                   std::back_inserter(weighted),
                   [](auto& gwy) {
                       return WeightedGateway{gwy, 1.0, false};
    });

    // Then filtered them by assigning a low weight to the generated gateways and a high weight to the provided gateways
    auto filtered = filter_generated_gateways(weighted, voronoiBuilder_->getVoronoiSkeleton());
    std::vector<Gateway> finalGateways;
    std::transform(filtered.begin(), filtered.end(), std::back_inserter(finalGateways), [](const auto& gwy) {
        return gwy.gateway;
    });

    return finalGateways;
}


SkeletonFeatures LocalTopoAreaEditor::computeGatewayFeatures(void) const
{
    createIsovistsIfNeeded();
    VoronoiEdges edges(voronoiBuilder_->getVoronoiSkeleton(), SKELETON_CELL_REDUCED_SKELETON);
    return extract_gateway_features_default(edges, *isovistField_, gatewayFeatureRadius_);
}


const VoronoiIsovistField& LocalTopoAreaEditor::isovistField(void) const
{
    createIsovistsIfNeeded();
    return *isovistField_;
}


void LocalTopoAreaEditor::constructHypotheses(const std::vector<Gateway>& gateways)
{
    createIsovistsIfNeeded();
    areaGraph_.reset(new AreaGraph(voronoiBuilder_->getVoronoiSkeleton(), gateways));
    hypothesisGraph_.reset(new HypothesisGraph(*areaGraph_,
                                               &(voronoiBuilder_->getVoronoiSkeleton()),
                                               isovistField_.get(),
                                               starBuilder_));
    initialHypothesisGraph_.reset(new HypothesisGraph(*areaGraph_,
                                                      &(voronoiBuilder_->getVoronoiSkeleton()),
                                                      isovistField_.get(),
                                                      starBuilder_));

    initialHypotheses_.clear();
    initialHypotheses_.insert(initialHypotheses_.end(),
                              initialHypothesisGraph_->beginHypothesis(),
                              initialHypothesisGraph_->endHypothesis());
    resetStoredHypotheses();
}


bool LocalTopoAreaEditor::labelArea(AreaHypothesis* area, HypothesisType label)
{
    if(!utils::contains(hypotheses_, area))
    {
        return false;
    }

    area->setType(label);

    auto initialIt = std::find_if(initialHypotheses_.begin(), initialHypotheses_.end(), [area](AreaHypothesis* h) {
        return h->amountContained(*area) == HypothesisContainment::complete;
    });

    if(initialIt != initialHypotheses_.end())
    {
        (*initialIt)->setType(label);
    }
    // Sanity check that the hypothesis not found in the initial hypotheses is a member of mergedHypotheses. Otherwise,
    // it should be in initialHypothese
    else
    {
        assert(!utils::contains_if(mergedHypotheses_, [area](const std::unique_ptr<AreaHypothesis>& h) {
            return h.get() == area;
        }));
    }

    return true;
}


std::size_t LocalTopoAreaEditor::labelRemainingAreas(HypothesisType label)
{
    // For each hypothesis that is a generic area, set the label to the provided type
    std::size_t numLabeled = 0;

    for(auto& hyp : hypotheses_)
    {
        if(hyp->getType() == HypothesisType::kArea)
        {
            hyp->setType(label);
            ++numLabeled;
        }
    }

    return numLabeled;
}


void LocalTopoAreaEditor::simplifyAreas(void)
{
    hypothesisGraph_->simplify(BoundaryMergeCondition::kSameType);
    resetStoredHypotheses();
}


bool LocalTopoAreaEditor::mergeAreas(const std::vector<AreaHypothesis*>& hypothesesToMerge)
{
    std::vector<const AreaHypothesisBoundary*> boundariesToMerge;

    // Using the full for-loop because the iterators are needed on the inside
    for(auto hypIt = hypothesesToMerge.begin(), hypEnd = hypothesesToMerge.end(); hypIt != hypEnd; ++hypIt)
    {
        add_merge_boundaries((*hypIt)->beginBoundary(),
                             (*hypIt)->endBoundary(),
                             hypIt + 1,
                             hypEnd,
                             boundariesToMerge);
    }

    if(!boundariesToMerge.empty())
    {
        mergedHypotheses_.emplace_back(new AreaHypothesis(boundariesToMerge, hypothesesToMerge.front()->getType()));
        hypotheses_.push_back(mergedHypotheses_.back().get());

        // Condition for checking if a hypothesis matches one of those that was just merged, and hence should be removed
        utils::erase_remove_if(hypotheses_, [&hypothesesToMerge](AreaHypothesis* hyp) {
            return utils::contains(hypothesesToMerge, hyp);
        });

        return true;
    }

    return false;
}


void LocalTopoAreaEditor::resetAreas(void)
{
    resetStoredHypotheses();

    for(auto& hyp : hypotheses_)
    {
        hyp->setType(HypothesisType::kArea);
    }
}


void LocalTopoAreaEditor::saveToLocalTopoMap(const std::string& filename) const
{
    auto topoMap = constructLocalTopoMap();

    if(topoMap && !utils::save_serializable_to_file(filename, *topoMap))
    {
        std::cerr << "ERROR:LocalTopoAreaEditor: Failed to save topo map to file " << filename << '\n';
    }
}


boost::optional<LocalTopoMap> LocalTopoAreaEditor::constructLocalTopoMap(void) const
{
    try
    {
        std::vector<AreaProposal> proposals =
                hypothesisGraph_->createProposalsForCurrentHypotheses(voronoiBuilder_->getVoronoiSkeleton());

        AreaBuilder builder(starBuilder_);
        LocalAreaVec areas;
        std::transform(proposals.begin(), proposals.end(), std::back_inserter(areas), [&](AreaProposal& p) {
                return builder.buildArea(p, *lpm_, voronoiBuilder_->getVoronoiSkeleton(), *isovistField_);
        });

        LocalTopoMap topoMap(1,
                            voronoiBuilder_->getVoronoiSkeleton().getTimestamp(),
                            areas,
                            voronoiBuilder_->getVoronoiSkeleton());
        return topoMap;
    }
    catch(InvalidAreaException& e)
    {
        std::cout << "ERROR: LocalTopoAreaEditor: Failed to create LocalTopoMap for current hypotheses." << e.what()
            << '\n';
        return boost::none;
    }
}


void LocalTopoAreaEditor::createIsovistsIfNeeded(void) const
{
    if(!isovistField_)
    {
        utils::isovist_options_t options;
        options.maxDistance = maxIsovistRange_;
        options.numRays = numIsovistRays_;
        options.saveEndpoints = false;

        isovistField_ = std::make_unique<VoronoiIsovistField>(voronoiBuilder_->getVoronoiSkeleton(),
                                                              IsovistLocation::REDUCED_SKELETON,
                                                              options);
    }
}


void LocalTopoAreaEditor::resetStoredHypotheses(void)
{
    hypotheses_.clear();
    mergedHypotheses_.clear();

    hypotheses_.insert(hypotheses_.end(), hypothesisGraph_->beginHypothesis(), hypothesisGraph_->endHypothesis());
    mergedHypotheses_.reserve(hypotheses_.size());
}


void add_merge_boundaries(AreaHypothesis::BoundaryIter    boundaryIt,
                          AreaHypothesis::BoundaryIter    boundaryEnd,
                          std::vector<AreaHypothesis*>::const_iterator hypBegin,
                          std::vector<AreaHypothesis*>::const_iterator hypEnd,
                          std::vector<const AreaHypothesisBoundary*>& toMerge)
{
    for(; boundaryIt != boundaryEnd; ++boundaryIt)
    {
        for(auto hypIt = hypBegin; hypIt != hypEnd; ++hypIt)
        {
            // Can break out of this loop if a match is found because the boundary can only have
            // one other hypothesis associated with it
            if((*boundaryIt)->containsHypothesis(*hypIt))
            {
                toMerge.push_back(*boundaryIt);
                break;
            }
        }
    }
}


} // namespace hssh
} // namespace vulcan
