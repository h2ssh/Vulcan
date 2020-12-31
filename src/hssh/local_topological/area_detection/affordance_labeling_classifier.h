/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_classifier.h
* \author   Collin Johnson
*
* Declaration of AffordanceLabelingClassifier.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AFFORDANCE_LABELING_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AFFORDANCE_LABELING_CLASSIFIER_H

#include "hssh/local_topological/area_detection/area_classifier.h"
#include "hssh/local_topological/event_visitor.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class  AreaBuilder;
class  AreaParser;
class  AreaProposal;
class  LocalArea;
class  LocalPerceptualMap;
class  SmallScaleStarBuilder;
class  VoronoiSkeletonGrid;
class  Gateway;
struct area_classifier_params_t;
struct local_area_debug_info_t;


const std::string kAffordanceLabelingClassifier("affordance-labeling");


/**
* AffordanceLabelingClassifier segments and classifies an LPM into a collection of LocalAreas representing the
* three major types of areas: path segments, decision points, and destinations.
*
* The area classification process involves three main steps:
*
* 1) Build the AreaGraph from the gateways and Voronoi skeleton.
* 2) Segment the AreaGraph into distinct AreaProposals.
* 3) Match the proposed areas to areas detected on previous updates.
*/
class AffordanceLabelingClassifier : public AreaClassifier,
                                     public LocalAreaEventVisitor
{
public:

    /**
    * Constructor for AffordanceLabelingClassifier.
    *
    * \param    params          Parameters for running the area classification algorithm
    * \param    mapName         Name of the map being classified (basename for learned classifiers)
    */
    AffordanceLabelingClassifier(const area_classifier_params_t& params, const std::string& mapName);

    /**
    * Destructor for AffordanceLabelingClassifier.
    */
    ~AffordanceLabelingClassifier(void);

    // AreaClassifier interface
    LabelingError classifyAreas(const std::vector<Gateway>& gateways,
                                const VoronoiSkeletonGrid&  skeleton,
                                const VoronoiIsovistField&  isovistField,
                                const LocalPerceptualMap&   lpm) override;
    void processAreaEvents(const LocalAreaEventVec& events) override;
    Areas currentAreas(void) const override;
    void  sendDebug(system::DebugCommunicator& communicator) const override;

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const AreaTransitionEvent& event) override;
    void visitTurnAround(const TurnAroundEvent& event) override;

private:

    // No value semantics
    AffordanceLabelingClassifier(void) = delete;
    AffordanceLabelingClassifier(const AffordanceLabelingClassifier& toCopy) = delete;
    void operator=(const AffordanceLabelingClassifier& rhs) = delete;

    std::shared_ptr<SmallScaleStarBuilder> starBuilder;
    std::shared_ptr<AreaBuilder> areaBuilder;
    std::unique_ptr<AreaParser>  parser;

    std::unique_ptr<local_area_debug_info_t> debug;     // used to break the dependency with debug_info.h, which has many includes and could trigger lots of recompilation

    Areas areas_;

    int64_t totalTime = 0;
    int numUpdates = 0;

    void createAreasFromProposals(const std::vector<AreaProposal>& proposals,
                                  const VoronoiSkeletonGrid& skeleton,
                                  const VoronoiIsovistField& isovists,
                                  const LocalPerceptualMap& lpm);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AFFORDANCE_LABELING_CLASSIFIER_H
