/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stability_analyzer.h
* \author   Collin Johnson
*
* Declaration of AreaStabilityAnalyzer.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_ANALYZER_H
#define HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_ANALYZER_H

#include "hssh/types.h"
#include "hssh/local_topological/evaluation/stability_log.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/event_visitor.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/small_scale_star.h"
#include "math/geometry/polygon.h"
#include "math/geometry/rectangle.h"
#include "utils/pose_trace.h"

namespace vulcan
{
namespace hssh
{

/**
* StableArea is a local area that's been projected into the global reference frame.
*/
struct StableArea
{
    AreaType type;
    SmallScaleStar star;            // for dest and decision only
    math::Rectangle<float> boundary;
    math::Polygon<float> polygon;
    pose_t transform;        // transform from event coords to global coords
    LocalArea::Ptr area;            // area associated with the stable area
};

/**
* AreaStabilityAnalyzer is used to evaluate the stability of the topological abstraction constructed by the area
* detection algorithm. The stability determines if the same abstraction is created for the same parts of an
* environment, regardless of where the area is entered from, the presence of pedestrians, or moving furniture and
* clutter.
*
* The stability analyzer works by projecting the detected areas into a single global reference frame. By doing so,
* drift in the local reference frame does not affect the ability to compare if two areas are the same. Furthermore,
* it checks that they are the same in the underlying environment of interest.
*/
class AreaStabilityAnalyzer : public LocalAreaEventVisitor
{
public:

    using area_iterator = std::vector<StableArea>::const_iterator;

    /**
    * Constructor for AreaStabilityAnalyzer.
    *
    * Creates an analyzer for events detected in the provided map
    *
    * \param    map         Map in which to analyze events
    */
    AreaStabilityAnalyzer(const LocalTopoMap& map);

    // Methods for adding new information to be analyzed -- self-explanatory
    void addLog(const AreaStabilityLog& log);
    void addLocalPose(const LocalPose& pose);
    void addGlobalPose(const GlobalPose& pose);

    /**
    * NOTE: Events should be added after their corresponding poses. The transform to the global reference
    * isn't guaranteed unless that's true.
    */
    void addEvent(const LocalAreaEventPtr& event);

    // Iterate through all stored areas corresponding to the added events
    std::size_t size(void) const { return areas_.size(); }
    area_iterator begin(void) const { return areas_.begin(); }
    area_iterator end(void) const { return areas_.end(); }

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const AreaTransitionEvent& event) override;
    void visitTurnAround    (const TurnAroundEvent&     event) override;

private:

    LocalTopoMap map_;
    utils::PoseTrace localPoses_;
    utils::PoseTrace globalPoses_;
    std::vector<StableArea> areas_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_EVALUATION_STABILITY_ANALYZER_H
