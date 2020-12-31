/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     areas.cpp
* \author   Collin Johnson
*
* Definition of constructors and methods in the LocalArea class hierarchy.
*/

#include "hssh/local_topological/area.h"
#include "utils/pose_trace.h"
#include <algorithm>
#include <sstream>

namespace vulcan
{
namespace hssh
{

LocalArea::LocalArea(int id, const AreaExtent& extent, const std::vector<Gateway>& gateways)
: id_(id)
, extent_(extent)
, gateways_(gateways)
{
}


math::Rectangle<double> LocalArea::boundary(math::ReferenceFrame frame) const
{
    return extent_.rectangleBoundary(frame);
}


math::Polygon<double> LocalArea::polygonBoundary(math::ReferenceFrame frame) const
{
    return extent_.polygonBoundary(frame);
}


bool LocalArea::contains(const Point<float>& point, math::ReferenceFrame frame) const
{
    return extent_.contains(point, frame);
}


bool LocalArea::containsCell(const Point<float>& position) const
{
    auto insideIt = std::find_if(extent_.begin(), 
                                 extent_.end(), 
                                 [position](const Point<double>& cell) {
            return ((position.x - cell.x) >= 0.0)
                && ((position.x - cell.x) < 0.05)
                && ((position.y - cell.y) >= 0.0)
                && ((position.y - cell.y) < 0.05);
    });
    
    return insideIt != extent_.end();
}


bool LocalArea::hasGateway(const Gateway& gateway) const
{
    return std::find(gateways_.begin(), gateways_.end(), gateway) != gateways_.end();
}


gateway_crossing_t LocalArea::findTransitionGateway(const pose_t& poseInArea,
                                                    const pose_t& poseOutOfArea) const
{
    // If the poses are the same, then a crossing couldn't have occurred because the robot didn't move!
    if(poseInArea == poseOutOfArea)
    {
        return {boost::none, 0};
    }

    Line<double>  poseLine(poseInArea.toPoint(), poseOutOfArea.toPoint());
    Point<double> intersectionPoint;

    for(auto& gateway : gateways_)
    {
        // An intersection must occur when one pose is on one side of the gateway and one pose is on the other
        if(gateway.intersectsWithBoundary(poseLine, intersectionPoint))
        {
            return { gateway, std::max(poseInArea.timestamp, poseOutOfArea.timestamp) };
        }
    }

    return {boost::none, 0};
}


bool operator==(const LocalArea& lhs, const LocalArea& rhs)
{
    const double AREA_OVERLAP_RATIO_THRESHOLD = 0.8;
    
    auto lhsBoundary = lhs.boundary(math::ReferenceFrame::GLOBAL);
    auto rhsBoundary = rhs.boundary(math::ReferenceFrame::GLOBAL);
    
    if((lhsBoundary.area() == 0.0) || (rhsBoundary.area() == 0.0))
    {
        return false;
    }
    
    auto intersection = lhsBoundary.intersection(rhsBoundary);
    
    if((intersection.area() / lhsBoundary.area() < AREA_OVERLAP_RATIO_THRESHOLD) ||
       (intersection.area() / rhsBoundary.area() < AREA_OVERLAP_RATIO_THRESHOLD))
    {
        return false;
    }
    
    return true;
}

} // namespace hssh
} // namespace vulcan
