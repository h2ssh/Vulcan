/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_planner_panel.h
* \author   Collin Johnson
*
* Declaration of GoalPlannerPanel.
*/

#ifndef UI_DEBUG_GOAL_PLANNER_PANEL_H
#define UI_DEBUG_GOAL_PLANNER_PANEL_H

#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_params.h"
#include "ui/common/ui_panel.h"
#include "planner/goal/messages.h"
#include "hssh/global_topological/topological_map.h"
#include "hssh/global_topological/utils/metric_map_cache.h"
#include "utils/mutex.h"
#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class GoalPlannerDisplayWidget;

/**
* goal_planner_panel_widgets_t specifies all the widgets belonging to the GoalPlannerPanel.
* These widgets are provided in the constructor and are then used for event handling, etc.
*/
struct goal_planner_panel_widgets_t
{
    GoalPlannerDisplayWidget* displayWidget;
    wxRadioBox*               representationBox;
    wxRadioBox*               routeDisplayBox;
    wxTextCtrl*               animateFPSText;
    
    goal_planner_panel_widgets_t(void)
        : displayWidget(0)
        , representationBox(0)
        , routeDisplayBox(0)
        , animateFPSText(0)
    {
    }
};

/**
* GoalPlannerPanel manages interaction with the global topo planner. The panel passes
* output from the planner to a GoalPlannerDisplayWidget for rendering. The panel supports
* setting the global location in a map and a target, thereby generating a plan. The panel also
* allows a user to Confirm or Cancel a planned route.
*/
class GoalPlannerPanel : public UIPanel
{
public:

    /**
    * Constructor for GoalPlannerPanel.
    *
    * \param    params          Parameters for the panel
    * \param    widgets         Widgets for the panel to use
    */
    GoalPlannerPanel(const ui_params_t& params, goal_planner_panel_widgets_t& widgets);

    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

    // Data handlers
    void handleData(const hssh::TopologicalMap&        map,     const std::string& channel);
    void handleData(const planner::GoalTarget&        target,   const std::string& channel);
    void handleData(const planner::GoalRoute&         route,    const std::string& channel);
    void handleData(const planner::GoalProgress&      progress, const std::string& channel);
    void handleData(const planner::goal_debug_info_t& info,     const std::string& channel);

private:

    // Event handlers
    void representationChanged(wxCommandEvent& event);
    void routeDisplayChanged  (wxCommandEvent& event);
    void setLocationPressed   (wxCommandEvent& event);
    void sendLocationPressed  (wxCommandEvent& event);
    void setGoalPressed       (wxCommandEvent& event);
    void sendGoalPressed      (wxCommandEvent& event);
    void animateSearchPressed (wxCommandEvent& event);
    void confirmRoutePressed  (wxCommandEvent& event);
    void cancelRoutePressed   (wxCommandEvent& event);

    void sendRouteCommand(planner::route_command_t command);

    GoalPlannerDisplayWidget* displayWidget;
    system::ModuleCommunicator*         consumer;

    int32_t currentRouteId;
    bool    routeIsConfirmed;

    hssh::TopologicalMap     map;
    hssh::MetricMapCache places;

    goal_planner_display_params_t params;

    utils::Mutex dataLock;

    wxRadioBox* representationBox;
    wxRadioBox* routeDisplayBox;
    wxTextCtrl* animateFPSText;
    
    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_GOAL_PLANNER_PANEL_H
