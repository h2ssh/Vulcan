/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     shrink_and_stretch.cpp
* \author   Collin Johnson
*
* Definition of ShrinkAndStretchBoundary.
*/

#include "hssh/local_topological/boundary/shrink_and_stretch.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"
#include "math/geometry/shape_fitting.h"
#include "utils/isovist.h"

namespace vulcan
{
namespace hssh
{

math::Rectangle<float> area_visible_from_position(Point<float> position, const LocalPerceptualMap& lpm);


boost::optional<SmallScaleSpaceBoundary::MapBoundary> ShrinkAndStretchBoundary::computeBoundary(
    const LocalAreaEventVec& events,
    const LocalTopoMap& topoMap,
    const LocalPose& pose,
    const LocalPerceptualMap& lpm)
{
    currentMap_ = &topoMap;
    currentLpm_ = &lpm;
    currentPosition_ = pose.pose().toPoint();
    updatedBoundary_ = boost::none;     // Initialize to no new boundary, as boundary-changing events may not have occurred

    for(auto& e : events)
    {
        e->accept(*this);
    }

    return updatedBoundary_;
}


void ShrinkAndStretchBoundary::visitAreaTransition(const AreaTransitionEvent& event)
{
    // Create a bounding box around the current area plus its immediate neighbors.
    auto currentArea = event.enteredArea();
    
    Point<float> boundaryRadius(3.0, 3.0); // buffer to give the new boundary

    if(currentArea)
    {
        auto areaPlusNeighbors = currentMap_->neighborsOf(*currentArea);
        areaPlusNeighbors.push_back(currentArea);

        // Accumulate the vertices for all the included areas
        std::vector<Point<float>> boundaryPoints;
        for(auto& area : areaPlusNeighbors)
        {
            auto boundary = area->boundary();
            boundaryPoints.push_back(boundary.bottomLeft - boundaryRadius);
            boundaryPoints.push_back(boundary.topRight + boundaryRadius);
        }

        auto visibleArea = area_visible_from_position(currentPosition_, *currentLpm_);
        boundaryPoints.push_back(visibleArea.bottomLeft);
        boundaryPoints.push_back(visibleArea.topRight);

        // And create an axis-aligned bounding rectangle around them
        updatedBoundary_ = math::axis_aligned_bounding_rectangle<float>(boundaryPoints.begin(),
                                                                        boundaryPoints.end());
    }
}


void ShrinkAndStretchBoundary::visitTurnAround(const TurnAroundEvent& event)
{
    // Nothing to be done for turn around events.
}


math::Rectangle<float> area_visible_from_position(Point<float> position, const LocalPerceptualMap& lpm)
{
    using namespace utils;
    isovist_options_t options;
    options.maxDistance = 15.0f;
    options.numRays = 1440;
    options.saveEndpoints = true;

    auto rayEndFunc = [](const LocalPerceptualMap& lpm, Point<int> cell) {
        return lpm.getCost(cell) > 127;
    };

    Isovist visibleArea(global_point_to_grid_cell_round(position, lpm), lpm, rayEndFunc, options);
    return math::axis_aligned_bounding_rectangle<float>(visibleArea.begin(), visibleArea.end());
}

} // namespace hssh
} // namespace vulcan
