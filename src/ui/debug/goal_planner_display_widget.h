/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_planner_display_widget.h
 * \author   Collin Johnson
 *
 * Declaration of GoalPlannerDisplayWidget.
 */

#ifndef UI_DEBUG_GOAL_PLANNER_DISPLAY_WIDGET_H
#define UI_DEBUG_GOAL_PLANNER_DISPLAY_WIDGET_H

#include "core/line.h"
#include "hssh/global_topological/global_location.h"
#include "hssh/global_topological/topological_map.h"
#include "planner/goal/debug_info.h"
#include "planner/goal/goal_progress.h"
#include "planner/goal/goal_route.h"
#include "planner/goal/goal_target.h"
#include "ui/common/ui_params.h"
#include "ui/components/open_gl_widget.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace hssh
{
class MetricMapCache;
}
namespace math
{
template <typename T>
class Rectangle;
}

namespace ui
{

class TopologicalMapRenderer;
class TopologicalGraphRenderer;

/**
 * GoalPlannerDisplayWidget renders data produced by the global topo planner. In particular,
 * the graph used for the search, the topological map, and the route and progress along the route
 * are rendered on the screen.
 */
class GoalPlannerDisplayWidget : public OpenGLWidget
{
public:
    enum widget_mode_t
    {
        VIEW_MAP,
        SELECT_LOCATION,
        SELECT_GOAL
    };

    enum map_representation_t
    {
        TOPOLOGICAL,
        GRAPH
    };

    enum route_display_t
    {
        ROUTE,
        PROGRESS
    };

    /**
     * Constructor for GoalPlannerDisplayWidget.
     *
     * \param    parent          Parent window for the widget
     */
    GoalPlannerDisplayWidget(wxWindow* parent,
                             wxWindowID id = wxID_ANY,
                             const wxPoint& pos = wxDefaultPosition,
                             const wxSize& size = wxDefaultSize,
                             long style = 0,
                             const wxString& name = wxString((const wxChar*)("GLCanvas")),
                             const wxPalette& palette = wxNullPalette);

    void setWidgetParams(const goal_planner_display_params_t& params);

    void setMode(widget_mode_t mode);

    void showRepresentation(map_representation_t representation) { this->representation = representation; }
    void showRoute(route_display_t display) { routeDisplay = display; }

    void animateSearch(int fps);   // TODO: How do I do this animation?

    // Methods to set the data to be rendered by the widget
    void setTopologicalMap(const hssh::TopologicalMap& map);
    void setGoalTarget(const planner::GoalTarget& target);
    void setGoalRoute(const planner::GoalRoute& route);
    void setGoalProgress(const planner::GoalProgress& progress);
    void setGoalDebugInfo(const planner::goal_debug_info_t& info);
    void setPlaceManager(const hssh::MetricMapCache* manager);

    // Query the selection process -- only if the mouse is up has a selection actually been made
    bool haveSelectedLocation(void) const { return locationIsValid; }
    hssh::GlobalLocation getSelectedLocation(void) const { return selectedLocation; }
    int32_t getSelectedGoal(void) const { return selectedGoalId; }

private:
    enum selection_state_t
    {
        HOVERING,
        MOUSE_DOWN,
        MOUSE_UP
    };

    // OpenGLWidget interface
    virtual void renderWidget(void);

    void setRouteColors(void);
    void setProgressColors(void);
    void setPathColors(void);
    void setSequenceColors(const std::vector<planner::goal_route_element_t>& sequence,
                           const GLColor& startColor,
                           const GLColor& endColor);
    void setElementColor(const planner::goal_route_element_t& element, const GLColor& color);

    // Mouse handlers for selecting locations and goals
    void handleMouseMotion(wxMouseEvent& event);
    void handleMouseLeftDown(wxMouseEvent& event);
    void handleMouseLeftUp(wxMouseEvent& event);

    // Return -1 if no place under the cursor
    void handlePlaceSelectionEvent(int mouseX, int mouseY);
    void handleGoalSelectionEvent(const Point<float>& cursor);
    void handleLocationSelectionEvent(const Point<float>& cursor);
    int32_t selectPlaceUnderCursor(const Point<float>& cursor);
    int32_t selectSegmentUnderCursor(const Point<float>& cursor);
    bool cursorIsInsideBoundary(const Point<float>& cursor,
                                const math::Rectangle<float>& boundary,
                                const pose_t& referenceFrame);
    void setPlaceSelectionColor(int32_t oldId, int32_t selectedId, float alpha);
    void setSegmentSelectionColor(int32_t oldId, int32_t selectedId, float alpha);
    void populatePathSegmentLines(void);
    void convertSelectedLocationsToGlobalLocation(void);


    widget_mode_t mode;
    map_representation_t representation;
    route_display_t routeDisplay;

    hssh::TopologicalMap map;
    planner::GoalTarget target;
    planner::GoalRoute route;
    planner::GoalProgress progress;
    planner::goal_debug_info_t info;

    hssh::GlobalLocation selectedLocation;
    bool locationIsValid;
    int32_t selectedGoalId;
    int32_t hoverId;
    bool hoverIsPlace;
    selection_state_t selectionState;

    bool firstSelectedWasPlace;
    int32_t firstSelectedLocation;
    int32_t secondSelectedLocation;

    std::unordered_map<uint32_t, Line<float>> pathSegmentLines;

    const hssh::MetricMapCache* placeManager;

    bool haveProgress;
    bool haveRoute;
    bool haveTarget;

    TopologicalMapRenderer* mapRenderer;
    TopologicalGraphRenderer* graphRenderer;

    goal_planner_display_params_t params;

    utils::Mutex dataLock;

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_GOAL_PLANNER_DISPLAY_WIDGET_H
