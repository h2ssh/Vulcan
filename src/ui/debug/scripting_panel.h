/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     scripting_panel.h
* \author   Collin Johnson
*
* Definition of ScriptingPanel.
*/

#ifndef UI_DEBUG_SCRIPTING_PANEL_H
#define UI_DEBUG_SCRIPTING_PANEL_H

#include "ui/common/ui_panel.h"
#include "core/pose.h"
#include "utils/mutex.h"
#include <wx/wx.h>
#include <wx/listctrl.h>
#include <atomic>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalPose; }
namespace mpepc { struct named_pose_t; }
namespace ui
{

class  PlannerScriptingWidget;
class  PoseSelector;
struct ui_params_t;

/**
* scripting_panel_widgets_t contains all the stateful widgets that the ScriptingPanel needs access to
* outside of receiving events. Those widgets, namely the buttons, that raise an event when pressed don't
* need to be stashed away.
*/
struct scripting_panel_widgets_t
{
    PlannerScriptingWidget* scriptingWidget;
    wxListCtrl*             targetSetList;
    wxTextCtrl*             targetNameText;
    wxRadioButton*          poseTaskButton;
    wxRadioButton*          elevatorTaskButton;
    wxTextCtrl*             targetPoseText;
    wxListBox*              scriptList;

    scripting_panel_widgets_t(void)
        : scriptingWidget(0)
        , targetSetList(0)
        , targetNameText(0)
        , poseTaskButton(0)
        , elevatorTaskButton(0)
        , targetPoseText(0)
        , scriptList(0)
    {
    }
};

/**
* ScriptingPanel
*/
class ScriptingPanel : public UIPanel
{
public:

    /**
    * Constructor for ScriptingPanel.
    */
    ScriptingPanel(const ui_params_t& params, const scripting_panel_widgets_t& widgets);

    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

    // Data handlers
    void handleData(const hssh::LocalPerceptualMap& map, const std::string& channel);
    void handleData(const hssh::LocalPose& pose, const std::string& channel);

private:

    scripting_panel_widgets_t     widgets;
    std::shared_ptr<PoseSelector> poseSelector;

    std::shared_ptr<hssh::LocalPerceptualMap> lpm;
    std::vector<mpepc::named_pose_t>          targets;
    std::vector<mpepc::named_pose_t>          scriptTargets;

    system::ModuleCommunicator* consumer;

    bool isSelectingTarget;
    pose_t selectedTargetPose;
    pose_t currentPose_;

    std::atomic<bool> shouldCaptureNextLPM;
    utils::Mutex      lpmLock;

    void initializeTargetSelection(void);
    void updateTargetSelection(void);
    void stopTargetSelection(void);

    void addNewTargetToSet(const pose_t& targetPose);
    void addTargetToList(const mpepc::named_pose_t& target);
    void eraseSelectedTargets(void);

    void removeTargetFromScript(std::size_t index);
    void updateScriptList(void);

    // Gets the selected rows in increasing order
    std::vector<std::size_t> getSelectedRows(void) const;

    // Event handlers
    void loadMapPressed               (wxCommandEvent& event);
    void captureMapPressed            (wxCommandEvent& event);
    void selectTargetPosePressed      (wxCommandEvent& event);
    void useCurrentPosePressed        (wxCommandEvent& event);
    void createTargetPressed          (wxCommandEvent& event);
    void eraseTargetPressed           (wxCommandEvent& event);
    void saveTargetsPressed           (wxCommandEvent& event);
    void loadTargetsPressed           (wxCommandEvent& event);
    void addTargetToScriptPressed     (wxCommandEvent& event);
    void removeTargetFromScriptPressed(wxCommandEvent& event);
    void saveScriptPressed            (wxCommandEvent& event);
    void loadScriptPressed            (wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_SCRIPTING_PANEL_H
