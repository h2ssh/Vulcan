/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_planner_panel.cpp
* \author   Collin Johnson
*
* Definition of DecisionPlannerPanel.
*/

#include <ui/debug/decision_planner_panel.h>
#include <ui/debug/debug_ui.h>
#include <ui/common/ui_params.h>
#include <ui/debug/decision_planner_display_widget.h>
#include <system/module_communicator.h>
#include <utils/auto_mutex.h>
#include <sstream>
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(DecisionPlannerPanel, wxEvtHandler)
    EVT_BUTTON(ID_FORWARD_TARGET_BUTTON,         DecisionPlannerPanel::forwardTargetPressed)
    EVT_BUTTON(ID_LEFT_TARGET_BUTTON,            DecisionPlannerPanel::leftTargetPressed)
    EVT_BUTTON(ID_RIGHT_TARGET_BUTTON,           DecisionPlannerPanel::rightTargetPressed)
    EVT_BUTTON(ID_BACK_TARGET_BUTTON,            DecisionPlannerPanel::backTargetPressed)
    EVT_BUTTON(ID_PATH_TARGET_START_BUTTON,      DecisionPlannerPanel::pathStartPressed)
    EVT_BUTTON(ID_PATH_TARGET_END_BUTTON,        DecisionPlannerPanel::pathEndPressed)
    EVT_BUTTON(ID_REMOVE_RELATIVE_TARGET_BUTTON, DecisionPlannerPanel::removeTargetPressed)
    EVT_BUTTON(ID_CLEAR_RELATIVE_TARGETS_BUTTON, DecisionPlannerPanel::clearTargetsPressed)
    EVT_BUTTON(ID_SEND_RELATIVE_TARGETS_BUTTON,  DecisionPlannerPanel::sendTargetsPressed)
END_EVENT_TABLE()


DecisionPlannerPanel::DecisionPlannerPanel(const ui_params_t& params, decision_planner_panel_widgets_t widgets)
    : haveUpdatedSequence(true)
    , widget(widgets.widget)
    , commandQueueList(widgets.commandQueueList)
    , placeState(widgets.placeState)
    , pathState(widgets.pathState)
    , consumer(0)
{
    widget->setWidgetParams(params.lpmParams, params.localTopoParams, params.decisionPlannerParams);
    
    assert(widget);
    assert(commandQueueList);
    assert(placeState);
    assert(pathState);
}


void DecisionPlannerPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widget->setRenderContext(context);
    widget->setStatusBar(statusBar);
}


void DecisionPlannerPanel::subscribe(system::ModuleCommunicator& producer)
{
    
}


void DecisionPlannerPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if(consumer)
    {
        this->consumer = consumer;
    }
}


void DecisionPlannerPanel::update(void)
{
    utils::AutoMutex autoLock(sequenceLock);
    
    if(haveUpdatedSequence)
    {
        populateCommandList();
        haveUpdatedSequence = false;
    }
    
    widget->Refresh();
}


void DecisionPlannerPanel::saveSettings(utils::ConfigFileWriter& config)
{
    
}


void DecisionPlannerPanel::loadSettings(const utils::ConfigFile& config)
{
    
}


void DecisionPlannerPanel::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    widget->setLPM(lpm);
}


void DecisionPlannerPanel::handleData(const pose_t& pose, const std::string& channel)
{
    widget->setPose(pose);
}


void DecisionPlannerPanel::handleData(const hssh::LocalPath& path, const std::string& channel)
{
    // Ignoring paths for now
}


void DecisionPlannerPanel::populateCommandList(void)
{
    commandQueueList->Clear();
}


void DecisionPlannerPanel::forwardTargetPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::leftTargetPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::rightTargetPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::backTargetPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::pathStartPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::pathEndPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::removeTargetPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::clearTargetsPressed(wxCommandEvent& event)
{
}


void DecisionPlannerPanel::sendTargetsPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR:DecisionPlannerPanel: Forgot to set the output consumer\n");
}

} // namespace ui
} // namespace vulcan
