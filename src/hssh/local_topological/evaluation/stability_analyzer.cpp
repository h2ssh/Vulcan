/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     stability_analyzer.cpp
 * \author   Collin Johnson
 *
 * Definition of AreaStabilityAnalyzer.
 */

#include "hssh/local_topological/evaluation/stability_analyzer.h"
#include "hssh/local_topological/area_visitor.h"
#include "hssh/local_topological/areas/decision_point.h"
#include "hssh/local_topological/areas/destination.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/events/turn_around.h"
#include "math/geometry/shape_fitting.h"
#include "utils/stub.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

class StableAreaCreator : public LocalAreaVisitor
{
public:
    StableAreaCreator(const pose_t& local, const pose_t& global, const LocalTopoMap& map);

    boost::optional<StableArea> createdArea(void) const;

    // LocalAreaVisitor interface
    void visitDestination(const LocalDestination& destination) override;
    void visitDecisionPoint(const LocalDecisionPoint& decision) override;
    void visitPathSegment(const LocalPathSegment& path) override;


private:
    const LocalTopoMap& map_;
    pose_t localToGlobal_;   // transform to get to global coords
    boost::optional<StableArea> area_;

    void visitPlace(const LocalPlace& place);
};

////////////////////     AreaStabilityAnalyzer implementation       //////////////////////////////

AreaStabilityAnalyzer::AreaStabilityAnalyzer(const LocalTopoMap& map) : map_(map)
{
}


void AreaStabilityAnalyzer::addLog(const AreaStabilityLog& log)
{
    for (auto& pose : boost::make_iterator_range(log.beginLocal(), log.endLocal())) {
        addLocalPose(pose);
    }

    for (auto& pose : boost::make_iterator_range(log.beginGlobal(), log.endGlobal())) {
        addGlobalPose(pose);
    }

    for (auto& event : boost::make_iterator_range(log.beginEvent(), log.endEvent())) {
        addEvent(event);
    }
}


void AreaStabilityAnalyzer::addLocalPose(const LocalPose& pose)
{
    localPoses_.addPose(pose.pose());
}


void AreaStabilityAnalyzer::addGlobalPose(const GlobalPose& pose)
{
    globalPoses_.addPose(pose.pose());
}


void AreaStabilityAnalyzer::addEvent(const LocalAreaEventPtr& event)
{
    event->accept(*this);
}


void AreaStabilityAnalyzer::visitAreaTransition(const AreaTransitionEvent& event)
{
    if (auto exited = event.exitedArea()) {
        auto local = localPoses_.poseAt(event.timestamp());
        auto global = globalPoses_.poseAt(event.timestamp());

        StableAreaCreator creator(local, global, map_);
        exited->accept(creator);

        if (auto stable = creator.createdArea()) {
            stable->area = exited;
            areas_.push_back(*stable);
        }
    }
}


void AreaStabilityAnalyzer::visitTurnAround(const TurnAroundEvent& event)
{
    // Ignoring turn around events
}


////////////////////     StableAreaCreator implementation       //////////////////////////////

StableAreaCreator::StableAreaCreator(const pose_t& local, const pose_t& global, const LocalTopoMap& map) : map_(map)
{
    double deltaTheta = angle_diff(global.theta, local.theta);   // how much to rotate?
    auto deltaPosition = rotate(local.toPoint(), deltaTheta);    // difference in position

    localToGlobal_.x = global.x - deltaPosition.x;
    localToGlobal_.y = global.y - deltaPosition.y;
    localToGlobal_.theta = deltaTheta;
}


boost::optional<StableArea> StableAreaCreator::createdArea(void) const
{
    return area_;
}


void StableAreaCreator::visitDestination(const LocalDestination& destination)
{
    visitPlace(destination);
}


void StableAreaCreator::visitDecisionPoint(const LocalDecisionPoint& decision)
{
    visitPlace(decision);
}


void StableAreaCreator::visitPathSegment(const LocalPathSegment& path)
{
    StableArea area;
    area.type = path.type();
    area.transform = localToGlobal_;

    area.polygon = path.extent().polygonBoundary(math::ReferenceFrame::GLOBAL);

    for (auto& p : area.polygon) {
        p = homogeneous_transform(p, localToGlobal_.x, localToGlobal_.y, localToGlobal_.theta);
    }

    area.boundary = math::minimum_area_bounding_rectangle(area.polygon.begin(), area.polygon.end(), area.polygon);

    area_ = area;
    //     PRINT_PRETTY_STUB();
}


void StableAreaCreator::visitPlace(const LocalPlace& place)
{
    StableArea area;
    area.type = place.type();
    area.transform = localToGlobal_;

    area.polygon = place.extent().polygonBoundary(math::ReferenceFrame::GLOBAL);

    for (auto& p : area.polygon) {
        p = homogeneous_transform(p, localToGlobal_.x, localToGlobal_.y, localToGlobal_.theta);
    }

    area.boundary = math::minimum_area_bounding_rectangle(area.polygon.begin(), area.polygon.end(), area.polygon);

    // Create a transformed version of the small-scale star too
    auto fragments = place.star().getAllFragments();
    for (auto& frag : fragments) {
        frag.gateway = frag.gateway.changeReferenceFrame(localToGlobal_, map_.voronoiSkeleton());
    }

    area.star = SmallScaleStar(fragments);

    area_ = area;
}

}   // namespace hssh
}   // namespace vulcan
