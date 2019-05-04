/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_planner_display_widget.cpp
* \author   Collin Johnson
*
* Definition of GoalPlannerDisplayWidget.
*/

#include <ui/debug/goal_planner_display_widget.h>
#include <ui/components/topological_map_renderer.h>
#include <ui/components/graph_view_topological_map_renderer.h>
#include <ui/components/topological_graph_renderer.h>
#include <ui/common/color_interpolator.h>
#include <ui/common/gl_utilities.h>
#include <hssh/global_topological/utils/metric_map_cache.h>
#include <math/geometry/rectangle.h>
#include <utils/auto_mutex.h>
#include <utils/stub.h>
#include <iostream>
#include <iomanip>
#include <cassert>

namespace vulcan
{
namespace ui
{

const float HOVER_ALPHA    = 0.5f;
const float SELECTED_ALPHA = 0.9f;

BEGIN_EVENT_TABLE(GoalPlannerDisplayWidget, OpenGLWidget)
    EVT_MOTION(GoalPlannerDisplayWidget::handleMouseMotion)
    EVT_LEFT_DOWN(GoalPlannerDisplayWidget::handleMouseLeftDown)
    EVT_LEFT_UP(GoalPlannerDisplayWidget::handleMouseLeftUp)
END_EVENT_TABLE()


GoalPlannerDisplayWidget::GoalPlannerDisplayWidget(wxWindow*        parent,
                                                   wxWindowID       id,
                                                   const wxPoint&   pos,
                                                   const wxSize&    size,
                                                   long             style,
                                                   const wxString&  name,
                                                   const wxPalette& palette)
    : OpenGLWidget(parent, id, pos, size, style, name, palette)
    , mode(VIEW_MAP)
    , representation(TOPOLOGICAL)
    , routeDisplay(ROUTE)
    , locationIsValid(false)
    , placeManager(0)
    , haveProgress(false)
    , haveRoute(false)
    , haveTarget(false)
{
    mapRenderer   = new GraphViewTopologicalMapRenderer();
    graphRenderer = new TopologicalGraphRenderer();

    setCameraFocalPoint(Point<float>(0.0f, 0.0f));
}


void GoalPlannerDisplayWidget::setWidgetParams(const goal_planner_display_params_t& params)
{
    mapRenderer->setRenderColors(params.placeVertexColor, params.pathSegmentVertexColor, params.globalLocationColor, params.edgeColor);
    graphRenderer->setRenderColor(params.placeVertexColor, params.pathSegmentVertexColor, params.edgeColor);

    this->params = params;
}


void GoalPlannerDisplayWidget::setMode(widget_mode_t mode)
{
    if(mode == SELECT_LOCATION)
    {
        locationIsValid        = false;
        firstSelectedWasPlace  = false;
        firstSelectedLocation  = -1;
        secondSelectedLocation = -1;

        if(pathSegmentLines.empty())
        {
            populatePathSegmentLines();
        }
    }
    else if(mode == SELECT_GOAL)
    {
        selectedGoalId = -1;
    }

    if(mode != VIEW_MAP)
    {
        selectionState = HOVERING;
    }

    this->mode = mode;
}


void GoalPlannerDisplayWidget::animateSearch(int fps)
{

}


void GoalPlannerDisplayWidget::setTopologicalMap(const hssh::TopologicalMap& map)
{
    utils::AutoMutex autoLock(dataLock);

    this->map = map;

    // New map means the old pathSegmentLine must be cleared out. If in selection location, then they'll
    // need to be repopulated immediately
    pathSegmentLines.clear();

    if(mode == SELECT_LOCATION)
    {
        populatePathSegmentLines();
    }
}


void GoalPlannerDisplayWidget::setGoalTarget(const planner::GoalTarget& target)
{
    utils::AutoMutex autoLock(dataLock);

    this->target = target;
    haveTarget   = true;
}


void GoalPlannerDisplayWidget::setGoalRoute(const planner::GoalRoute& route)
{
    utils::AutoMutex autoLock(dataLock);

    this->route = route;
    haveRoute   = true;
}


void GoalPlannerDisplayWidget::setGoalProgress(const planner::GoalProgress& progress)
{
    utils::AutoMutex autoLock(dataLock);

    this->progress = progress;
    haveProgress   = true;
}


void GoalPlannerDisplayWidget::setGoalDebugInfo(const planner::goal_debug_info_t& info)
{
    utils::AutoMutex autoLock(dataLock);

    this->info = info;
}


void GoalPlannerDisplayWidget::setPlaceManager(const hssh::MetricMapCache* manager)
{
    assert(manager);

    utils::AutoMutex autoLock(dataLock);

    placeManager = manager;
}


void GoalPlannerDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock);

    assert(placeManager);

    if(mode == VIEW_MAP)
    {
        mapRenderer->resetColors();
        graphRenderer->resetColors();

        switch(routeDisplay)
        {
        case PROGRESS:
            if(haveProgress)
            {
                setProgressColors();
            }
            break;

        case ROUTE:
        default:
            if(haveRoute)
            {
                setRouteColors();
                setPathColors();
            }
            break;
        }

        if(haveTarget)
        {
            mapRenderer->setPlaceColor(target.getTargetPlace().id(), params.globalTargetColor);
        }
    }

    math::Rectangle<float> boundary;

//     switch(representation)
//     {
//     case GRAPH:
//         graphRenderer->render(info.searchInfo.graph);
//         break;
// 
//     case TOPOLOGICAL:
//         boundary = mapRenderer->calculateRenderedBoundary(map, *placeManager);
//         mapRenderer->renderTopoMap(map, *placeManager);
//         break;
//     }
    
    PRINT_PRETTY_STUB();

    Point<float> boundaryCenter((boundary.bottomLeft.x + boundary.topRight.x) / 2,
                                      (boundary.bottomLeft.y + boundary.topRight.y) / 2);

    setCameraFocalPoint(boundaryCenter);
    setViewRegion(boundary.width()*1.25f, boundary.height()*1.25f);
}


void GoalPlannerDisplayWidget::setRouteColors(void)
{
    setSequenceColors(route.getSequence(), params.routeElementColor, params.globalTargetColor);
}


void GoalPlannerDisplayWidget::setProgressColors(void)
{
    setSequenceColors(progress.getVisitedElements(),   params.visitedElementColor, params.visitedElementColor);
    setSequenceColors(progress.getRemainingElements(), params.activeElementColor,  params.remainingElementColor);
    setElementColor  (progress.getActiveElement(),     params.activeElementColor);
}


void GoalPlannerDisplayWidget::setPathColors(void)
{
    const std::vector<hssh::TopologicalVertex>& sequence = info.searchInfo.path.getPath();

    for(auto vertexIt = sequence.begin(), vertexEnd = sequence.end(); vertexIt != vertexEnd; ++vertexIt)
    {
        graphRenderer->setVertexColor(vertexIt->getId(), params.graphPathColor);
    }
}


void GoalPlannerDisplayWidget::setSequenceColors(const std::vector<planner::goal_route_element_t>& sequence, const GLColor& startColor, const GLColor& endColor)
{
    LinearColorInterpolator interpolator(startColor, endColor);

    float increment    = sequence.empty() ? 0.0f : 1.0f / sequence.size();
    float elementColor = 0.0f;

    for(auto elementIt = sequence.begin(), elementEnd = sequence.end(); elementIt != elementEnd; ++elementIt)
    {
        setElementColor(*elementIt, interpolator.calculateColor(elementColor));

        elementColor += increment;
    }
}


void GoalPlannerDisplayWidget::setElementColor(const planner::goal_route_element_t& element, const GLColor& color)
{
    if(element.isPlace)
    {
        mapRenderer->setPlaceColor(element.place.id(), color);
    }
    else // if(element.isSegment)
    {
        mapRenderer->setPathSegmentColor(element.segment.id(), color);
    }
}


void GoalPlannerDisplayWidget::handleMouseMotion(wxMouseEvent& event)
{
    if(mode == VIEW_MAP)
    {
        event.Skip();
    }
    else
    {
        handlePlaceSelectionEvent(event.GetX(), event.GetY());
    }
}


void GoalPlannerDisplayWidget::handleMouseLeftDown(wxMouseEvent& event)
{
    if(mode == VIEW_MAP)
    {
        event.Skip();
    }
    else
    {
        selectionState = MOUSE_DOWN;

        handlePlaceSelectionEvent(event.GetX(), event.GetY());
    }
}


void GoalPlannerDisplayWidget::handleMouseLeftUp(wxMouseEvent& event)
{
    if(mode == VIEW_MAP)
    {
        event.Skip();
    }
    else
    {
        selectionState = MOUSE_UP;

        handlePlaceSelectionEvent(event.GetX(), event.GetY());

        selectionState = HOVERING;
    }
}


void GoalPlannerDisplayWidget::handlePlaceSelectionEvent(int mouseX, int mouseY)
{
    wxSize size = GetSize();

    Point<float> cursorWorldCoordinate = convert_screen_to_world_coordinates(Point<int>(mouseX, size.GetHeight() - mouseY),
                                                                                   getCameraPosition());

    switch(mode)
    {
    case SELECT_GOAL:
        handleGoalSelectionEvent(cursorWorldCoordinate);
        break;

    case SELECT_LOCATION:
        handleLocationSelectionEvent(cursorWorldCoordinate);
        break;

    case VIEW_MAP:
    default:
        break;
    }
}


void GoalPlannerDisplayWidget::handleGoalSelectionEvent(const Point<float>& cursor)
{
    int32_t oldHoverId     = hoverId;
    int32_t oldSelectionId = selectedGoalId;

    hoverId = selectPlaceUnderCursor(cursor);

    if(selectionState != HOVERING)
    {
        selectedGoalId = hoverId;
    }

    setPlaceSelectionColor(oldHoverId,     hoverId,        HOVER_ALPHA);
    setPlaceSelectionColor(oldSelectionId, selectedGoalId, SELECTED_ALPHA);
}


void GoalPlannerDisplayWidget::handleLocationSelectionEvent(const Point<float>& cursor)
{
    int32_t oldHoverId           = hoverId;
    int32_t oldSecondSelectionId = secondSelectedLocation;

    int32_t selectedPlaceId   = selectPlaceUnderCursor(cursor);
    int32_t selectedSegmentId = selectSegmentUnderCursor(cursor);

    if(selectionState == MOUSE_UP)
    {
        if((firstSelectedLocation == -1) && ((selectedPlaceId != -1) || (selectedSegmentId != -1)))
        {
            firstSelectedWasPlace = selectedPlaceId != -1;
            firstSelectedLocation = firstSelectedWasPlace ? selectedPlaceId : selectedSegmentId;
        }
        else if(firstSelectedWasPlace)
        {
            secondSelectedLocation = selectedSegmentId;
            convertSelectedLocationsToGlobalLocation();
        }
        else if(selectedPlaceId != -1)
        {
            secondSelectedLocation = selectedPlaceId;
            convertSelectedLocationsToGlobalLocation();
        }
    }

    hoverId = (selectedPlaceId != -1) ? selectedPlaceId : selectedSegmentId;

    if(hoverIsPlace)
    {
        setPlaceSelectionColor(oldHoverId, -1, HOVER_ALPHA);
    }
    else
    {
        setSegmentSelectionColor(oldHoverId, -1, HOVER_ALPHA);
    }

    hoverIsPlace = selectedPlaceId != -1;

    if(hoverIsPlace)
    {
        setPlaceSelectionColor(-1, hoverId, HOVER_ALPHA);
    }
    else
    {
        setSegmentSelectionColor(-1, hoverId, HOVER_ALPHA);
    }

    if(firstSelectedWasPlace)
    {
        setPlaceSelectionColor(-1, firstSelectedLocation, SELECTED_ALPHA);
        setSegmentSelectionColor(oldSecondSelectionId, secondSelectedLocation, SELECTED_ALPHA);
    }
    else
    {
        setSegmentSelectionColor(-1, firstSelectedLocation, SELECTED_ALPHA);
        setPlaceSelectionColor(oldSecondSelectionId, secondSelectedLocation, SELECTED_ALPHA);
    }

}


int32_t GoalPlannerDisplayWidget::selectPlaceUnderCursor(const Point<float>& cursor)
{
    PRINT_PRETTY_STUB()
    return -1;
    
//     const auto& places = map.places();
// 
//     for(auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt)
//     {
//         const hssh::LocalPlace& local = placeManager->load(placeIt->second.getMetricPlaceId());
// 
//         if(cursorIsInsideBoundary(cursor, local.extent().boundary, placeIt->second.referenceFrame()))
//         {
//             return placeIt->second.getId();
//         }
//     }
// 
//     return -1;
}


int32_t GoalPlannerDisplayWidget::selectSegmentUnderCursor(const Point<float>& cursor)
{
    if(pathSegmentLines.empty())
    {
        return -1;
    }

    typedef std::pair<int, Line<float>> IdLinePair;

    auto lineComp = [&cursor](const IdLinePair& lhs, const IdLinePair& rhs)
                                { return distance_to_line_segment(cursor, lhs.second) < distance_to_line_segment(cursor, rhs.second); };

    auto closestLine = std::min_element(pathSegmentLines.begin(), pathSegmentLines.end(), lineComp);

    if(distance_to_line_segment(cursor, closestLine->second) < 1.5f)
    {
        return closestLine->first;
    }
    else
    {
        return -1;
    }
}


bool GoalPlannerDisplayWidget::cursorIsInsideBoundary(const Point<float>& cursor, const math::Rectangle<float>& boundary, const pose_t& referenceFrame)
{
    Point<float>     placeCenter = referenceFrame.toPoint();
    math::Rectangle<float> shiftedBoundary(boundary.bottomLeft + placeCenter, boundary.topRight + placeCenter);

    return shiftedBoundary.contains(cursor);
}


void GoalPlannerDisplayWidget::setPlaceSelectionColor(int32_t oldId, int32_t selectedId, float alpha)
{
    if(oldId != -1)
    {
        mapRenderer->resetPlaceColor(oldId);
    }

    if(selectedId != -1)
    {
        GLColor& placeColor = (mode == SELECT_LOCATION) ? params.globalLocationColor : params.globalTargetColor;
        mapRenderer->setPlaceColor(selectedId, GLColor(placeColor.red(), placeColor.green(), placeColor.blue(), alpha));
    }
}


void GoalPlannerDisplayWidget::setSegmentSelectionColor(int32_t oldId, int32_t selectedId, float alpha)
{
    if(oldId != -1)
    {
        mapRenderer->resetPathSegmentColor(oldId);
    }

    if(selectedId != -1)
    {
        GLColor& segmentColor = (mode == SELECT_LOCATION) ? params.globalLocationColor : params.globalTargetColor;
        mapRenderer->setPathSegmentColor(selectedId, GLColor(segmentColor.red(), segmentColor.green(), segmentColor.blue(), alpha));
    }
}


void GoalPlannerDisplayWidget::populatePathSegmentLines(void)
{
    PRINT_PRETTY_STUB()
    
//     auto places = map.decisionPoints();
//     auto paths  = map.paths();
// 
//     for(auto pathIt = paths.begin(), pathEnd = paths.end(); pathIt != pathEnd; ++pathIt)
//     {
//         const std::deque<hssh::GlobalPathSegment>& segments = pathIt->second->getGlobalPathSegments();
// 
//         for(auto segmentIt = segments.begin(), segmentEnd = segments.end(); segmentIt != segmentEnd; ++segmentIt)
//         {
//             if(!segmentIt->isFrontier())
//             {
//                 const hssh::GlobalPlace& plusPlace  = places.find(segmentIt->getPlusTransition().placeId)->second;
//                 const hssh::GlobalPlace& minusPlace = places.find(segmentIt->getMinusTransition().placeId)->second;
// 
//                 pathSegmentLines.insert(std::make_pair(segmentIt->getUniqueId(),
//                                                        Line<float>(plusPlace.referenceFrame().toPoint(),
//                                                                          minusPlace.referenceFrame().toPoint())));
//             }
//         }
//     }
}


void GoalPlannerDisplayWidget::convertSelectedLocationsToGlobalLocation(void)
{
    PRINT_PRETTY_STUB()
    
//     if(!map.getDecisionPoint(firstSelectedWasPlace ? firstSelectedLocation : secondSelectedLocation) ||
//        !map.getPathSegment(firstSelectedWasPlace ? secondSelectedLocation : firstSelectedLocation))
//     {
//         std::cerr<<"ERROR:GoalPlannerDisplayWidget:Invalid global location selected.\n";
//         return;
//     }
// 
//     const hssh::GlobalDecisionPoint* place   = map.getDecisionPoint(firstSelectedWasPlace ? firstSelectedLocation : secondSelectedLocation);
//     const hssh::GlobalPathSegment* segment = map.getPathSegment(firstSelectedWasPlace ? secondSelectedLocation : firstSelectedLocation);
// 
//     if(firstSelectedWasPlace)
//     {
//         selectedLocation = hssh::GlobalLocation(*place, *segment);
//     }
//     else
//     {
//         selectedLocation = hssh::GlobalLocation(*segment, *place);
// 
//         const hssh::GlobalPath* path = map.getPath(selectedLocation.pathId);
//         selectedLocation.pathState.segmentIndex = path->getSegmentIndex(path->getSegmentWithId(segment->getUniqueId()));
//     }
// 
//     locationIsValid = true;
}

} // namespace ui
} // namespace vulcan
