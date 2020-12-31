/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_planner_panel.cpp
* \author   Collin Johnson
*
* Definition of GoalPlannerPanel.
*/

#include "ui/debug/goal_planner_panel.h"
#include "ui/debug/goal_planner_display_widget.h"
#include "ui/debug/debug_ui.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/stub.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(GoalPlannerPanel, wxEvtHandler)
    EVT_RADIOBOX(ID_MAP_REPRESENTATION_RADIO_BOX,   GoalPlannerPanel::representationChanged)
    EVT_RADIOBOX(ID_GLOBAL_ROUTE_DISPLAY_RADIO_BOX, GoalPlannerPanel::routeDisplayChanged)
    EVT_BUTTON(ID_SET_LOCATION_BUTTON,              GoalPlannerPanel::setLocationPressed)
    EVT_BUTTON(ID_SEND_LOCATION_BUTTON,             GoalPlannerPanel::sendLocationPressed)
    EVT_BUTTON(ID_SET_GOAL_BUTTON,                  GoalPlannerPanel::setGoalPressed)
    EVT_BUTTON(ID_SEND_GOAL_BUTTON,                 GoalPlannerPanel::sendGoalPressed)
    EVT_BUTTON(ID_ANIMATE_SEARCH_BUTTON,            GoalPlannerPanel::animateSearchPressed)
    EVT_BUTTON(ID_CONFIRM_ROUTE_BUTTON,             GoalPlannerPanel::confirmRoutePressed)
    EVT_BUTTON(ID_CANCEL_ROUTE_BUTTON,              GoalPlannerPanel::cancelRoutePressed)
END_EVENT_TABLE()


GoalPlannerPanel::GoalPlannerPanel(const ui_params_t& params, goal_planner_panel_widgets_t& widgets)
    : displayWidget(widgets.displayWidget)
    , params(params.goalPlannerParams)
    , representationBox(widgets.representationBox)
    , routeDisplayBox(widgets.routeDisplayBox)
    , animateFPSText(widgets.animateFPSText)
{
    // Verify valid data for all the widgets
    assert(displayWidget);
    assert(representationBox);
    assert(routeDisplayBox);
    assert(animateFPSText);
    
    displayWidget->setWidgetParams(params.goalPlannerParams);
    displayWidget->setPlaceManager(&places);
}


void GoalPlannerPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    displayWidget->setRenderContext(context);
}


void GoalPlannerPanel::subscribe(system::ModuleCommunicator& producer)
{
}


void GoalPlannerPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if(consumer)
    {
        this->consumer = consumer;
    }
}


void GoalPlannerPanel::update(void)
{
    displayWidget->Refresh();
}


void GoalPlannerPanel::saveSettings(utils::ConfigFileWriter& config)
{
    
}


void GoalPlannerPanel::loadSettings(const utils::ConfigFile& config)
{
    
}


void GoalPlannerPanel::handleData(const hssh::TopologicalMap& map, const std::string& channel)
{
    this->map = map;
    displayWidget->setTopologicalMap(map);
}


void GoalPlannerPanel::handleData(const planner::GoalTarget& target, const std::string& channel)
{
    displayWidget->setGoalTarget(target);
}


void GoalPlannerPanel::handleData(const planner::GoalRoute& route, const std::string& channel)
{
    displayWidget->setGoalRoute(route);
}


void GoalPlannerPanel::handleData(const planner::GoalProgress& progress, const std::string& channel)
{
    displayWidget->setGoalProgress(progress);
}


void GoalPlannerPanel::handleData(const planner::goal_debug_info_t& info, const std::string& channel)
{
    displayWidget->setGoalDebugInfo(info);
}


void GoalPlannerPanel::representationChanged(wxCommandEvent& event)
{
    switch(representationBox->GetSelection())
    {
    case 0:
        displayWidget->showRepresentation(GoalPlannerDisplayWidget::TOPOLOGICAL);
        break;

    case 1:
        displayWidget->showRepresentation(GoalPlannerDisplayWidget::GRAPH);
        break;

    default:
        assert("Unknown representation selection" && false);
    }
}


void GoalPlannerPanel::routeDisplayChanged(wxCommandEvent& event)
{
    switch(routeDisplayBox->GetSelection())
    {
    case 0:
        displayWidget->showRoute(GoalPlannerDisplayWidget::ROUTE);
        break;

    case 1:
        displayWidget->showRoute(GoalPlannerDisplayWidget::PROGRESS);
        break;

    default:
        assert("Unknown route display selection" && false);
    }
}


void GoalPlannerPanel::setLocationPressed(wxCommandEvent& event)
{
    displayWidget->setMode(GoalPlannerDisplayWidget::SELECT_LOCATION);
}


void GoalPlannerPanel::sendLocationPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: Forgot to set output consumer in GoalPlannerPanel\n");

    displayWidget->setMode(GoalPlannerDisplayWidget::VIEW_MAP);

    if(displayWidget->haveSelectedLocation())
    {
        PRINT_PRETTY_STUB()
    }
}


void GoalPlannerPanel::setGoalPressed(wxCommandEvent& event)
{
    displayWidget->setMode(GoalPlannerDisplayWidget::SELECT_GOAL);
}


void GoalPlannerPanel::sendGoalPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: Forgot to set output consumer in GoalPlannerPanel\n");

    utils::AutoMutex autoLock(dataLock);

    int32_t goalId = displayWidget->getSelectedGoal();

    std::cout<<"Selected goal:"<<goalId<<'\n';

//     if((goalId != -1) && map.getPlace(goalId))
//     {
//         planner::GoalTarget target(goalId, map.id(), *map.getPlace(goalId));
//     }
    PRINT_PRETTY_STUB()

    displayWidget->setMode(GoalPlannerDisplayWidget::VIEW_MAP);
}


void GoalPlannerPanel::animateSearchPressed(wxCommandEvent& event)
{

}


void GoalPlannerPanel::confirmRoutePressed(wxCommandEvent& event)
{
    if(!routeIsConfirmed && (currentRouteId >= 0))
    {
        sendRouteCommand(planner::CONFIRM_ROUTE);
    }
}


void GoalPlannerPanel::cancelRoutePressed(wxCommandEvent& event)
{
    if(currentRouteId >= 0)
    {
        sendRouteCommand(planner::CANCEL_ROUTE);
    }
}


void GoalPlannerPanel::sendRouteCommand(planner::route_command_t command)
{
    planner::goal_route_command_message_t message;
    message.timestamp = utils::system_time_us();
    message.planId    = currentRouteId;
    message.command   = command;
    message.source    = std::string("DebugUI");

}

} // namespace ui
} // namespace vulcan
