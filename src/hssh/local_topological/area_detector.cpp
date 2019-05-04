/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_detector.cpp
* \author   Collin Johnson
*
* Definition of AreaDetector.
*/

#include <hssh/local_topological/area_detector.h>
#include <hssh/local_topological/area_detection/area_classifier.h>
#include <hssh/local_topological/area_detection/gateways/gateway_locator.h>
#include <hssh/local_topological/area_detection/voronoi_graph_builder.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/cmd_line.h>
#include <hssh/local_topological/debug_info.h>
#include <hssh/local_topological/event_visitor.h>
#include <hssh/local_topological/events/area_transition.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/pose.h>
#include <system/debug_communicator.h>
#include <utils/command_line.h>
#include <utils/stub.h>
#include <utils/timestamp.h>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <iostream>

namespace vulcan
{
namespace hssh
{

void populate_points_of_interest(const LocalTopoMap& topoMap, std::vector<Point<float>>& points);


// A simple visitor for sending transition events to a GatewayLocator
struct TransitionVisitor : public LocalAreaEventVisitor
{
    GatewayLocator* locator;

    void visitAreaTransition(const AreaTransitionEvent& event)
    {
        auto transition = event.transitionGateway();
        if(transition)
        {
            locator->assignTransitionGateway(*transition);
        }

        auto exitedArea = event.exitedArea();
        if(exitedArea)
        {
            locator->assignExitedAreaGateways(exitedArea->gateways());
        }
    }

    void visitTurnAround(const TurnAroundEvent& event) { } // do nothing here
};

AreaDetector::AreaDetector(const utils::ConfigFile& config,
                           const utils::CommandLine& cmdLine,
                           const std::string& mapName)
{
    area_detector_params_t params(config);

    if(cmdLine.argumentExists(kConstraintLogProbArg))
    {
        params.classifierParams.mcmcParams.failingConstraintLogProb = std::strtod(cmdLine.argumentValue(kConstraintLogProbArg).c_str(), 0);
    }

    if(cmdLine.argumentExists(kRepeatLogProbArg))
    {
        params.classifierParams.mcmcParams.repeatConfigDecreaseLogProb = std::strtod(cmdLine.argumentValue(kRepeatLogProbArg).c_str(), 0);
    }

    if(cmdLine.argumentExists(kMaxIterationsArg))
    {
        params.classifierParams.mcmcParams.maxIterations = std::strtol(cmdLine.argumentValue(kMaxIterationsArg).c_str(), 0, 10);
    }

    if(cmdLine.argumentExists(kSamplesPerIterArg))
    {
        params.classifierParams.mcmcParams.samplesPerIteration = std::strtol(cmdLine.argumentValue(kSamplesPerIterArg).c_str(), 0, 10);
    }

    skeletonBuilder_.reset(new VoronoiSkeletonBuilder(params.skeletonParams, params.prunerParams));
    gatewayLocator_.reset(new GatewayLocator(params.locatorParams, mapName));
    areaClassifier_ = create_area_classifier(params.classifierParams.classifierType, mapName, params.classifierParams);

    shouldBuildSkeleton_ = params.shouldBuildSkeleton;
    shouldComputeIsovists_ = params.shouldComputeIsovists;
    shouldFindGateways_  = params.shouldFindGateways;

    maxIsovistRange_ = params.maxIsovistRange;
    numIsovistRays_ = params.numIsovistRays;
}


AreaDetector::~AreaDetector(void)
{
    // For std::unique_ptr
}


AreaDetectorResult AreaDetector::detectAreas(const LocalPose& pose, const LocalPerceptualMap& map)
{
    if(shouldBuildSkeleton_)
    {
        skeletonBuilder_->buildVoronoiSkeleton(map, pose.pose(), pointsOfInterest_);
        pointsOfInterest_.clear();
    }

    utils::isovist_options_t options;
    options.maxDistance = maxIsovistRange_;
    options.numRays = numIsovistRays_;
    options.saveEndpoints = false;

    int64_t gatewayStart = utils::system_time_us();
    // If no isovist field should be constructed, pass it an empty cell vector instead of the Voronoi mask
    auto isovistField = shouldComputeIsovists_ ?
        VoronoiIsovistField(skeletonBuilder_->getVoronoiSkeleton(),
                            IsovistLocation::REDUCED_SKELETON,
                            options) :
        VoronoiIsovistField(skeletonBuilder_->getVoronoiSkeleton(),
                            CellVector(),
                            options);

    if(shouldFindGateways_)
    {
        gatewayLocator_->locateGateways(skeletonBuilder_->getVoronoiSkeleton(), isovistField);
    }
    gatewayTotalTime_ += utils::system_time_us() - gatewayStart;

    if(!gatewayLocator_->isTransitionGatewayValid())
    {
        ++gwyFailCount_;

        // If we fail too many times, then something is awry with the transition gateway, so toss it out.
        if(gwyFailCount_ > 2)
        {
            // The transition is invalid so no gateways should remain between iterations
            gatewayLocator_->assignFinalGateways(std::vector<Gateway>());
            gatewayLocator_->discardMostRecentTransitionGateway();
        }
        return AreaDetectorResult(LabelingError::invalid_transition_gateway);
    }
    else
    {
        gwyFailCount_ = 0;
    }

    int64_t classifyStart = utils::system_time_us();
    LabelingError classifyResult = areaClassifier_->classifyAreas(gatewayLocator_->getGateways(),
                                                                  skeletonBuilder_->getVoronoiSkeleton(),
                                                                  isovistField,
                                                                  map);
    parsingTotalTime_ += utils::system_time_us() - classifyStart;
    ++numUpdates_;

    std::cout << "INFO:AreaDetector: Timing: Gateway: " << (gatewayTotalTime_ / numUpdates_ / 1000) << "ms Classify: "
        << (parsingTotalTime_ / numUpdates_ / 1000) << "ms\n";

    // If the classification was successful, then create the LocalTopoMap and set the final set of gateways
    if(classifyResult == LabelingError::success)
    {
        auto areas = areaClassifier_->currentAreas();
        LocalTopoMap ltMap(1, map.getTimestamp(), areas.first, skeletonBuilder_->getVoronoiSkeleton(), areas.second);
        gatewayLocator_->assignFinalGateways(ltMap.gateways());
        populate_points_of_interest(ltMap, pointsOfInterest_);
        return AreaDetectorResult(ltMap);
    }
    else
    {
        // Otherwise, classification failed, so no gateways should remain between iterations
        gatewayLocator_->assignFinalGateways(std::vector<Gateway>());
        return AreaDetectorResult(classifyResult);
    }
}


void AreaDetector::processAreaEvents(const LocalAreaEventVec& events)
{
    // Inform the classifier when a new event occurs
    areaClassifier_->processAreaEvents(events);

    // Inform the gateway locator of any possible transitions that have occurred
    TransitionVisitor visitor;
    visitor.locator = gatewayLocator_.get();

    for(auto& e : events)
    {
        e->accept(visitor);
    }
}


void AreaDetector::resetAreas(void)
{
    PRINT_STUB("AreaDetector::resetAreas(void)");
}


void AreaDetector::sendDebug(system::DebugCommunicator& communicator)
{
    communicator.sendDebug(skeletonBuilder_->getVoronoiSkeleton());
    communicator.sendDebug(gatewayLocator_->getDebugInfo());
    areaClassifier_->sendDebug(communicator);
}


void populate_points_of_interest(const LocalTopoMap& topoMap, std::vector<Point<float>>& points)
{
    // All gateways are points of interest, so just use those
    for(auto& gwy : topoMap.gateways())
    {
        points.push_back(gwy.center());
    }
}

} // namespace hssh
} // namespace vulcan
