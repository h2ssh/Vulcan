/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map_renderer.h.cpp
* \author   Collin Johnson
*
* Definition of TopologicalMapRenderer.
*/

#include "ui/components/topological_map_renderer.h"
#include "hssh/global_topological/state.h"
#include "hssh/global_topological/topological_map.h"
#include "hssh/local_topological/areas/place.h"
#include "hssh/global_topological/utils/metric_map_cache.h"
#include "math/geometry/shape_fitting.h"
#include "ui/common/gl_shapes.h"
#include "utils/stub.h"
#include <GL/gl.h>
#include <boost/range/adaptor/map.hpp>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace ui
{

const float PATH_DIRECTION_DISTANCE = 2.0f;

void TopologicalMapRenderer::setRenderColors(const GLColor& placeColor,
                                             const GLColor& pathColor,
                                             const GLColor& locationColor,
                                             const GLColor& frontierColor)
{
    this->placeColor    = placeColor;
    this->pathColor     = pathColor;
    this->locationColor = locationColor;
    this->frontierColor = frontierColor;
}


void TopologicalMapRenderer::setPlaceColor(int id, const GLColor& color)
{
    placeColors[id] = color;
}


void TopologicalMapRenderer::setPathSegmentColor(uint32_t id, const GLColor& color)
{
    segmentColors[id] = color;
}


void TopologicalMapRenderer::resetPlaceColor(int id)
{
    auto placeIt = placeColors.find(id);

    if(placeIt != placeColors.end())
    {
        placeColors.erase(placeIt);
    }
}


void TopologicalMapRenderer::resetPathSegmentColor(uint32_t id)
{
    auto segmentIt = segmentColors.find(id);

    if(segmentIt != segmentColors.end())
    {
        segmentColors.erase(segmentIt);
    }
}


void TopologicalMapRenderer::resetColors(void)
{
    placeColors.clear();
    segmentColors.clear();
}


math::Rectangle<float> TopologicalMapRenderer::calculateRenderedBoundary(const hssh::TopologicalState& topoMap,
                                                                         const hssh::MetricMapCache& places)
{
    std::vector<Point<float>> mapPoints;

    for(auto& p : boost::adaptors::values(topoMap.map->places()))
    {
        assert(p);
        auto frame = topoMap.map->referenceFrame(p->id());
        mapPoints.push_back(frame.toPoint());

        auto metric = places.loadMap(p->id());
        if(metric)
        {
            auto boundary = metric->rectangleBoundary();
            mapPoints.push_back(homogeneous_transform(boundary.bottomLeft, frame.x, frame.y, frame.theta));
            mapPoints.push_back(homogeneous_transform(boundary.topRight, frame.x, frame.y, frame.theta));
        }
    }

    if(!mapPoints.empty())
    {
        return math::axis_aligned_bounding_rectangle<float>(mapPoints.begin(), mapPoints.end());
    }
    else
    {
        return math::Rectangle<float>(Point<float>(-10, -10), Point<float>(10, 10));
    }
}


void TopologicalMapRenderer::renderTopoMap(const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places)
{
    drawPlaces(topoMap, places);
    drawPaths(topoMap, places);
//     drawLocation(topoMap, places);
}


void TopologicalMapRenderer::renderPathSegment(const hssh::GlobalPathSegment& segment,
                                               const map_place_info_t&        plusPlace,
                                               const map_place_info_t&        minusPlace,
                                               path_segment_attribute_t       attributes)
{
    assert(!segment.isFrontier());

    const float BORDER_WIDTH   = 3.0f;
    const float CONTROL_LENGTH = 3.0f;

    pose_t plusLocation  = plusPlace.referenceFrame;
    pose_t minusLocation = minusPlace.referenceFrame;

    auto plusTransition  = segment.plusTransition();
    auto minusTransition = segment.minusTransition();

    glLineWidth(BORDER_WIDTH);

    GLColor segmentColor = getSegmentColor(segment, attributes);
    segmentColor.set();

    // If the loaded small-scale stars have the appropriate fragments and it is a self-loop, i.e. path with same area
    // at both ends, then render the segment between the places
    // as a Bezier curve based on the path directions
    // Otherwise, just draw a straight line between the two places
    if(plusPlace.topo->cycle().contains(plusTransition)
        && minusPlace.topo->cycle().contains(minusTransition)
        && plusPlace.metric
        && minusPlace.metric)
//         && (plusTransition.otherArea(segment.toArea()) == minusTransition.otherArea(segment.toArea())))
    {
        hssh::local_path_fragment_t plusFragment  = plusPlace.metric->star().getFragmentWithId(
            plusPlace.topo->cycle().transitionIndex(plusTransition));
        hssh::local_path_fragment_t minusFragment = minusPlace.metric->star().getFragmentWithId(
            minusPlace.topo->cycle().transitionIndex(minusTransition));

        double plusDirection = plusFragment.gateway.direction() + plusLocation.theta;
        // Path goes from plus to minus, so the minus gateway always points into the path
        double minusDirection = minusFragment.gateway.direction() + minusLocation.theta;// + M_PI;

        float length = segment.lambda().magnitude();

        GLfloat controlPoints[4][3];
        controlPoints[0][0] = plusLocation.x;
        controlPoints[0][1] = plusLocation.y;
        controlPoints[0][2] = 0.0f;

        controlPoints[1][0] = plusLocation.x + (std::cos(plusDirection) * length / CONTROL_LENGTH);
        controlPoints[1][1] = plusLocation.y + (std::sin(plusDirection) * length / CONTROL_LENGTH);
        controlPoints[1][2] = 0.0f;

        controlPoints[2][0] = minusLocation.x + (std::cos(minusDirection) * length / CONTROL_LENGTH);
        controlPoints[2][1] = minusLocation.y + (std::sin(minusDirection) * length / CONTROL_LENGTH);
        controlPoints[2][2] = 0.0f;

        controlPoints[3][0] = minusLocation.x;
        controlPoints[3][1] = minusLocation.y;
        controlPoints[3][2] = 0.0f;

        glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &controlPoints[0][0]);
        glEnable(GL_MAP1_VERTEX_3);
        glMapGrid1f(30, 0.0f, 1.0f);
        glEvalMesh1(GL_LINE, 0, 30);
        glEnd();
    }
    else
    {
        glBegin(GL_LINES);
        glVertex2f(plusLocation.x, plusLocation.y);
        glVertex2f(minusLocation.x, minusLocation.y);
        glEnd();
    }
}


void TopologicalMapRenderer::renderFrontierPathSegment(const hssh::GlobalPathSegment& segment,
                                                       const map_place_info_t&        plusPlace,
                                                       const map_place_info_t&        minusPlace,
                                                       path_segment_attribute_t       attributes)
{
    // Don't really know where the frontiers are, so just ignore them.
    return;

//     const float BORDER_WIDTH = 3.0f;
//     const auto& place = plusPlace.topo ? plusPlace : minusPlace;
//     hssh::GlobalTransition transition = segment.plusTransition();
//     if(!plusPlace.topo)
//     {
//         transition = segment.minusTransition();
//     }
//
//     if(!place.topo) // need the topological information for the path segment rendering
//     {
//         return;
//     }
//
//     std::size_t index = place.topo->cycle().transitionIndex(transition);
//     pose_t location = place.referenceFrame;
//     pose_t frontierLocation;
//
//     if(place.metric && place.metric->star().hasFragmentWithId(index))
//     {
//         auto entryFragment = place.metric->star().getFragmentWithId(index);
//         auto gatewayCenter = math::convert_reference_frame(entryFragment.gateway.center(),
//                                                            math::ReferenceFrame::GLOBAL,
//                                                            math::ReferenceFrame::LOCAL,
//                                                            place.metric->extent().center());
//         auto gatewayDirection = entryFragment.gateway.direction() - place.metric->extent().center().theta;
//         frontierLocation.x = gatewayCenter.x + (std::cos(gatewayDirection) * PATH_DIRECTION_DISTANCE);
//         frontierLocation.y = gatewayCenter.y + (std::sin(gatewayDirection) * PATH_DIRECTION_DISTANCE);
//     }
//     else
//     {
//         frontierLocation = location;
//     }
//
//     glPushMatrix();
//     glTranslatef(location.x, location.y, 0.0f);
//     glRotatef(location.theta * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
//     glLineWidth(BORDER_WIDTH);
//     glBegin(GL_LINES);
//     frontierColor.set();
//     glVertex2f(0.0f, 0.0f);
//     glVertex2f(frontierLocation.x, frontierLocation.y);
//     glEnd();
//     glPopMatrix();
}


GLColor TopologicalMapRenderer::getPlaceColor(const map_place_info_t& place, place_attribute_t attributes)
{
    if(attributes == CURRENT_PLACE)
    {
        return locationColor;
    }

    auto colorIt = placeColors.find(place.topo->id());

    if(colorIt != placeColors.end())
    {
        return colorIt->second;
    }

    return placeColor;
}


GLColor TopologicalMapRenderer::getSegmentColor(const hssh::GlobalPathSegment& segment, path_segment_attribute_t attributes)
{
    if(attributes == CURRENT_PATH)
    {
        return locationColor;
    }

    auto colorIt = segmentColors.find(segment.id());

    if(colorIt != segmentColors.end())
    {
        return colorIt->second;
    }

    return pathColor;
}


void TopologicalMapRenderer::drawPlaces(const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places)
{
    for(auto& place : boost::adaptors::values(topoMap.map->places()))
    {
        map_place_info_t info = create_place_info(place->id(), topoMap, places);
        bool isCurrentPlace = topoMap.location.areaId == place->id();
        renderPlace(info, (isCurrentPlace ? CURRENT_PLACE : NORMAL_PLACE));
    }
}


void TopologicalMapRenderer::drawPaths(const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places)
{
    for(auto& segment : boost::adaptors::values(topoMap.map->segments()))
    {
        drawPathSegment(*segment, topoMap, places);
    }
}


void TopologicalMapRenderer::drawPathSegment(const hssh::GlobalPathSegment& segment,
                                             const hssh::TopologicalState& topoMap,
                                             const hssh::MetricMapCache& places)
{
    bool isCurrentPath = segment.id() == topoMap.location.areaId;

    map_place_info_t minus = create_place_info(segment.minusPlace().id(), topoMap, places);
    map_place_info_t plus  = create_place_info(segment.plusPlace().id(),  topoMap, places);

    if(segment.isFrontier())
    {
        renderFrontierPathSegment(segment, plus, minus, isCurrentPath ? CURRENT_PATH : NORMAL_PATH);
    }
    else
    {
        renderPathSegment(segment, plus, minus, isCurrentPath ? CURRENT_PATH : NORMAL_PATH);
    }
}


void TopologicalMapRenderer::drawLocation(const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places)
{
    // Ignore being at a frontier place when drawing the location
    if(topoMap.location.areaId == hssh::kFrontierId)
    {
        return;
    }

    if(topoMap.location.areaType != hssh::AreaType::path_segment)
    {
        drawPlaceLocation(topoMap.location, create_place_info(topoMap.location.areaId, topoMap, places));
    }
    // Is there actually a place we came from or are going to? If not, then the mapping is still bootstrapping and thus there is
    // no location to draw
    else if(auto segment = topoMap.map->getPathSegment(topoMap.location.areaId))
    {
        drawPathLocation(topoMap.location,
                         create_place_info(segment->minusPlace().id(),  topoMap, places),
                         create_place_info(segment->plusPlace().id(), topoMap, places));
    }
}


void TopologicalMapRenderer::drawPlaceLocation(const hssh::GlobalLocation& location, const map_place_info_t& place)
{
    // Drawing the location requires the topo place to exist
    if(!place.topo)
    {
        return;
    }

    pose_t pose = place.referenceFrame;

    if(place.metric)
    {
        const auto& cycle = place.topo->cycle();
        const auto& small = place.metric->star();

        auto exitDirection = cycle.aligned(location.entryTransition);
        auto exitIndex = cycle.transitionIndex(exitDirection);
        if(small.hasFragmentWithId(exitIndex))
        {
            // Gateways always point outbound, so reverse direction since we're in the place
            pose.theta += small.getFragmentWithId(exitIndex).gateway.direction();
        }
    }

    renderRobotTriangle(pose);
}


void TopologicalMapRenderer::drawPathLocation(const hssh::GlobalLocation& location,
                                              const map_place_info_t& entry,
                                              const map_place_info_t& target)
{
    const map_place_info_t& info = entry.topo ? entry : target;
    // When starting on a path segment, there won't be any entry information
    if(!info.topo)
    {
        return;
    }

    const auto& cycle = info.topo->cycle();

    pose_t pose;

    if(info.metric)
    {
        const auto& small = info.metric->star();

        auto exitIndex = cycle.transitionIndex(location.entryTransition);

        if(small.hasFragmentWithId(exitIndex))
        {
            hssh::local_path_fragment_t smallFragment  = small.getFragmentWithId(exitIndex);
            auto gatewayCenter = math::convert_reference_frame(smallFragment.gateway.center(),
                                                               math::ReferenceFrame::GLOBAL,
                                                               math::ReferenceFrame::LOCAL,
                                                               info.metric->center());
            auto gatewayDirection = angle_diff(smallFragment.gateway.direction(), info.metric->center().theta);

            pose.theta = gatewayDirection;
            pose.x = gatewayCenter.x;
            pose.y = gatewayCenter.y;
        }
    }

    pose.x += info.referenceFrame.x + PATH_DIRECTION_DISTANCE*std::cos(pose.theta);
    pose.y += info.referenceFrame.y + PATH_DIRECTION_DISTANCE*std::sin(pose.theta);

    renderRobotTriangle(pose);
}


void TopologicalMapRenderer::renderRobotTriangle(const pose_t& pose)
{
    const float WIDTH  = 0.7f;
    const float HEIGHT = 1.2f;

    glColor4f(0.0f, 0.0f, 0.0f, 0.3f);
    gl_draw_filled_triangle(pose.toPoint(), WIDTH, HEIGHT, pose.theta);
    glColor4f(0.0f, 0.0f, 0.0f, 0.75f);
    gl_draw_line_triangle  (pose.toPoint(), WIDTH, HEIGHT, pose.theta, 2.0f);
}


void TopologicalMapRenderer::updateMapBoundary(const pose_t& placeCenter, const math::Rectangle<float>& placeBoundary)
{
    // If the mapBoundary has zero area, then this is the initial place being added, so the map boundary is the same as the place boundary
    if(!boundaryIsInitialized)
    {
        Point<float> placePosition = placeCenter.toPoint();

        mapBoundary.bottomLeft  = placeBoundary.bottomLeft + placePosition;
        mapBoundary.bottomRight = placeBoundary.bottomRight + placePosition;
        mapBoundary.topLeft     = placeBoundary.topLeft + placePosition;
        mapBoundary.topRight    = placeBoundary.topRight + placePosition;

        boundaryIsInitialized = true;
    }
    else
    {
        if(placeBoundary.bottomLeft.x + placeCenter.x < mapBoundary.bottomLeft.x)
        {
            mapBoundary.bottomLeft.x = placeBoundary.bottomLeft.x + placeCenter.x;
            mapBoundary.topLeft.x    = mapBoundary.bottomLeft.x;
        }

        if(placeBoundary.bottomLeft.y + placeCenter.y < mapBoundary.bottomLeft.y)
        {
            mapBoundary.bottomLeft.y  = placeBoundary.bottomLeft.y + placeCenter.y;
            mapBoundary.bottomRight.y = mapBoundary.bottomLeft.y;
        }

        if(placeBoundary.topRight.x + placeCenter.x > mapBoundary.topRight.x)
        {
            mapBoundary.topRight.x    = placeBoundary.topRight.x + placeCenter.x;
            mapBoundary.bottomRight.x = mapBoundary.topRight.x;
        }

        if(placeBoundary.topRight.y + placeCenter.y > mapBoundary.topRight.y)
        {
            mapBoundary.topRight.y = placeBoundary.topRight.y + placeCenter.y;
            mapBoundary.topLeft.y  = mapBoundary.topRight.y;
        }
    }
}


TopologicalMapRenderer::map_place_info_t
TopologicalMapRenderer::create_place_info(hssh::Id placeId,
                                          const hssh::TopologicalState& topoMap,
                                          const hssh::MetricMapCache& places)
{
    map_place_info_t info;
    info.referenceFrame = topoMap.map->referenceFrame(placeId);
    info.topo = topoMap.map->getPlace(placeId);

    if(info.topo)
    {
        // See if the map exists in the cache and can therefore be loaded and used for rendering
        auto metricMap = places.loadMap(info.topo->metricPlaceId());

        if(metricMap)
        {
            auto localArea = metricMap->localArea();
            if((localArea->type() == hssh::AreaType::decision_point)
                || (localArea->type() == hssh::AreaType::destination))
            {
                info.metric = static_cast<const hssh::LocalPlace*>(localArea);
            }
            else
            {
                info.metric = nullptr;
            }
        }
    }

    return info;
}

} // namespace ui
} // namespace vulcan
