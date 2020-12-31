/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_planner_panel.h
* \author   Collin Johnson
*
* Declaration of DecisionPlannerPanel.
*/

#ifndef UI_DEBUG_LOCAL_TOPO_PLANNER_PANEL_H
#define UI_DEBUG_LOCAL_TOPO_PLANNER_PANEL_H

#include <wx/wx.h>
#include "utils/mutex.h"
#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_panel.h"

class wxGrid;
class wxListBox;

namespace vulcan
{
namespace ui
{

class  DecisionPlannerDisplayWidget;
struct ui_params_t;

/**
* decision_planner_panel_widgets_t defines all the widgets used in the Local Topo Planner tab
* in the DebugUI. These widgets are manipulated by the DecisionPlannerPanel class, and thus
* need to be provided to a panel on construction.
*/
struct decision_planner_panel_widgets_t
{
    DecisionPlannerDisplayWidget* widget;

    wxListBox* commandQueueList;
    wxGrid*    placeState;
    wxGrid*    pathState;
};

/**
* DecisionPlannerPanel provides event handling and text updating for the Decision Planner panel.
* Data is received here and passed on to the DecisionPlannerDisplayWidget. The target sequence
* will be written into the command list.
*
* A target sequence can be constructed and sent to the
*/
class DecisionPlannerPanel : public UIPanel
{
public:

    /**
    * Constructor for DecisionPlannerPanel.
    *
    * \param    params          Parameters for the UI
    * \param    widgets         Widgets used in the panel
    */
    DecisionPlannerPanel(const ui_params_t& params, decision_planner_panel_widgets_t widgets);
    
    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

    // LPMDataConsumer interface
    virtual void handleData(const hssh::LocalPerceptualMap& lpm,  const std::string& channel);
    virtual void handleData(const pose_t&            pose, const std::string& channel);

    // LocalTopologyDataConsumer interface
    virtual void handleData(const hssh::LocalPath& path,   const std::string& channel);

private:

    void populateCommandList(void);

    // Event handlers
    void forwardTargetPressed(wxCommandEvent& event);
    void leftTargetPressed   (wxCommandEvent& event);
    void rightTargetPressed  (wxCommandEvent& event);
    void backTargetPressed   (wxCommandEvent& event);
    void pathStartPressed    (wxCommandEvent& event);
    void pathEndPressed      (wxCommandEvent& event);
    void removeTargetPressed (wxCommandEvent& event);
    void clearTargetsPressed (wxCommandEvent& event);
    void sendTargetsPressed  (wxCommandEvent& event);

    bool haveUpdatedSequence;

    DecisionPlannerDisplayWidget* widget;

    wxListBox* commandQueueList;
    wxGrid*    placeState;
    wxGrid*    pathState;

    system::ModuleCommunicator* consumer;

    utils::Mutex sequenceLock;

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_LOCAL_TOPO_PLANNER_PANEL_H
