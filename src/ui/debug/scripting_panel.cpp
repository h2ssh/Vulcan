/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     scripting_panel.cpp
* \author   Collin Johnson
*
* Implementation of ScriptingPanel.
*/

#include "ui/debug/scripting_panel.h"
#include "ui/debug/planner_scripting_widget.h"
#include "ui/debug/debug_ui.h"
#include "ui/common/metric_path_creator.h"
#include "ui/common/ui_params.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "hssh/local_metric/pose.h"
#include "mpepc/metric_planner/script/script.h"
#include "mpepc/metric_planner/script/target_set.h"
#include "utils/auto_mutex.h"
#include <cassert>
#include <fstream>
#include <sstream>


// Separate namespace for event table to help KDevelop parsing
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(ScriptingPanel, wxEvtHandler)
    EVT_BUTTON(ID_SCRIPTING_LOAD_MAP_BUTTON,        ScriptingPanel::loadMapPressed)
    EVT_BUTTON(ID_SCRIPTING_CAPTURE_MAP_BUTTON,     ScriptingPanel::captureMapPressed)
    EVT_BUTTON(ID_SCRIPT_TARGET_SELECT_POSE_BUTTON, ScriptingPanel::selectTargetPosePressed)
    EVT_BUTTON(ID_SCRIPT_TARGET_CURRENT_BUTTON,     ScriptingPanel::useCurrentPosePressed)
    EVT_BUTTON(ID_SCRIPT_CREATE_TARGET_BUTTON,      ScriptingPanel::createTargetPressed)
    EVT_BUTTON(ID_SCRIPT_ERASE_TARGET_BUTTON,       ScriptingPanel::eraseTargetPressed)
    EVT_BUTTON(ID_SCRIPT_SAVE_TARGETS_BUTTON,       ScriptingPanel::saveTargetsPressed)
    EVT_BUTTON(ID_SCRIPT_LOAD_TARGETS_BUTTON,       ScriptingPanel::loadTargetsPressed)
    EVT_BUTTON(ID_SCRIPT_ADD_TARGET_BUTTON,         ScriptingPanel::addTargetToScriptPressed)
    EVT_BUTTON(ID_SCRIPT_REMOVE_TARGET_BUTTON,      ScriptingPanel::removeTargetFromScriptPressed)
    EVT_BUTTON(ID_SCRIPT_SAVE_BUTTON,               ScriptingPanel::saveScriptPressed)
    EVT_BUTTON(ID_SCRIPT_LOAD_BUTTON,               ScriptingPanel::loadScriptPressed)
END_EVENT_TABLE()

}
}

namespace vulcan
{
namespace ui
{

ScriptingPanel::ScriptingPanel(const ui_params_t& params, const scripting_panel_widgets_t& widgets)
    : widgets(widgets)
    , poseSelector(new PoseSelector)
{
    assert(widgets.scriptingWidget);
    assert(widgets.elevatorTaskButton);
    assert(widgets.poseTaskButton);
    assert(widgets.scriptList);
    assert(widgets.targetNameText);
    assert(widgets.targetPoseText);
    assert(widgets.targetSetList);

    widgets.scriptingWidget->setParams(params);

    widgets.targetSetList->InsertColumn(0, wxT("Name"));
    widgets.targetSetList->InsertColumn(1, wxT("Pose"));
    widgets.targetSetList->InsertColumn(2, wxT("Task"));
}


void ScriptingPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets.scriptingWidget->setRenderContext(context);
    widgets.scriptingWidget->setStatusBar(statusBar);
}


void ScriptingPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalPerceptualMap>(this);
    producer.subscribeTo<hssh::LocalPose>(this);
}


void ScriptingPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if(consumer)
    {
        this->consumer = consumer;
    }
}


void ScriptingPanel::update(void)
{
    if(isSelectingTarget)
    {
        updateTargetSelection();
    }

    {
        utils::AutoMutex autoLock(lpmLock);
        widgets.scriptingWidget->setCurrentPose(currentPose_);
    }

    widgets.scriptingWidget->Refresh();
}


void ScriptingPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void ScriptingPanel::loadSettings(const utils::ConfigFile& config)
{
}


void ScriptingPanel::handleData(const hssh::LocalPerceptualMap& map, const std::string& channel)
{
    if(shouldCaptureNextLPM)
    {
        utils::AutoMutex autoLock(lpmLock);

        lpm.reset(new hssh::LocalPerceptualMap(map));
        widgets.scriptingWidget->setLPM(lpm);
        shouldCaptureNextLPM = false;
    }
}


void ScriptingPanel::handleData(const hssh::LocalPose& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(lpmLock);

    currentPose_ = pose.pose();
}


void ScriptingPanel::initializeTargetSelection(void)
{
    if(!isSelectingTarget)
    {
        poseSelector->reset();
        widgets.scriptingWidget->pushMouseHandler(poseSelector.get());
        isSelectingTarget = true;
    }
}


void ScriptingPanel::updateTargetSelection(void)
{
    selectedTargetPose = poseSelector->getSelectedTarget();
    std::ostringstream poseString;
    poseString << selectedTargetPose;
    widgets.targetPoseText->ChangeValue(wxString(poseString.str().c_str(), wxConvUTF8));

    if(poseSelector->hasSelectedTarget())
    {
        widgets.scriptingWidget->showSelectedTarget(true);
        widgets.scriptingWidget->setSelectedTarget(poseSelector->getSelectedTarget());
    }

    widgets.scriptingWidget->showHoverTarget(true);
    widgets.scriptingWidget->setHoverTarget(poseSelector->getHoverTarget());
}


void ScriptingPanel::stopTargetSelection(void)
{
    if(isSelectingTarget)
    {
        widgets.scriptingWidget->removeMouseHandler(poseSelector.get());
        isSelectingTarget = false;
    }

    widgets.scriptingWidget->clearHoverTarget();
    widgets.scriptingWidget->clearSelectedTarget();
    widgets.scriptingWidget->showHoverTarget(false);
    widgets.scriptingWidget->showSelectedTarget(false);
}


void ScriptingPanel::addNewTargetToSet(const pose_t& targetPose)
{
    mpepc::named_pose_t target;

    target.name = widgets.targetNameText->GetValue().mb_str();
    target.pose = targetPose;

// TODO: this needs to get done somewhere
//     target.taskType = widgets.poseTaskButton->GetValue() ? planner::PoseTargetTask::POSE_TARGET_TASK_ID : planner::ElevatorTask::ELEVATOR_TASK_ID;

    targets.push_back(target);
    widgets.scriptingWidget->setCompletedTargets(targets);
    addTargetToList(target);
}


void ScriptingPanel::addTargetToList(const mpepc::named_pose_t& target)
{
    long nextItemIndex = widgets.targetSetList->GetItemCount(); // append to the end

    std::ostringstream poseString;
    poseString << target.pose;

    wxString taskString;
//     switch(target.taskType)
//     {
//     case planner::PoseTargetTask::POSE_TARGET_TASK_ID:
//         taskString = wxT("TARGET");
//         break;
//
//     case planner::ElevatorTask::ELEVATOR_TASK_ID:
//         taskString = wxT("ELEVATOR");
//         break;
//
//     default:
//         taskString = wxT("UNKNOWN");
//     }
    taskString = wxT("TARGET");

    long itemId = widgets.targetSetList->InsertItem(nextItemIndex, wxString(target.name.c_str(), wxConvUTF8));

    widgets.targetSetList->SetItem(itemId, 1, wxString(poseString.str().c_str(), wxConvUTF8));
    widgets.targetSetList->SetItem(itemId, 2, taskString);
}


void ScriptingPanel::eraseSelectedTargets(void)
{
    auto selectedRows = getSelectedRows();

    // Erase the selected targets from the internal structure, then rebuild the listctrl with the new targets
    // rather than deleting items from the list control
    for(int n = selectedRows.size(); --n >= 0;)
    {
        targets.erase(targets.begin() + selectedRows[n]);
        widgets.targetSetList->DeleteItem(selectedRows[n]);
    }

    widgets.scriptingWidget->setCompletedTargets(targets);  // changed the number of targets, so need to update the display
}


void ScriptingPanel::removeTargetFromScript(std::size_t index)
{
    if(index < scriptTargets.size())
    {
        scriptTargets.erase(scriptTargets.begin() + index);
    }

    updateScriptList();
}


void ScriptingPanel::updateScriptList(void)
{
    widgets.scriptList->Clear();

    for(auto& target : scriptTargets)
    {
        widgets.scriptList->Append(wxString(target.name.c_str(), wxConvUTF8));
    }
}


std::vector<std::size_t> ScriptingPanel::getSelectedRows(void) const
{
    std::vector<std::size_t> selectedRows;

    long nextSelectedIndex = widgets.targetSetList->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);

    while(nextSelectedIndex != -1)
    {
        selectedRows.push_back(nextSelectedIndex);
        nextSelectedIndex = widgets.targetSetList->GetNextItem(nextSelectedIndex, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
    }

    return selectedRows;
}


void ScriptingPanel::loadMapPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.scriptingWidget, wxT("Select map file..."), wxT(""), wxT(""), wxT("*.lpm"), wxFD_OPEN);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        wxString path = loadDialog.GetPath();

        lpm.reset(new hssh::LocalPerceptualMap());
        hssh::load_lpm_1_0(std::string(path.mb_str()), *lpm);
        widgets.scriptingWidget->setLPM(lpm);
    }
}


void ScriptingPanel::captureMapPressed(wxCommandEvent& event)
{
    shouldCaptureNextLPM = true;
}


void ScriptingPanel::selectTargetPosePressed(wxCommandEvent& event)
{
    initializeTargetSelection();
}


void ScriptingPanel::useCurrentPosePressed(wxCommandEvent& event)
{
    addNewTargetToSet(currentPose_);
}


void ScriptingPanel::createTargetPressed(wxCommandEvent& event)
{
    stopTargetSelection();
    addNewTargetToSet(selectedTargetPose);
}


void ScriptingPanel::eraseTargetPressed(wxCommandEvent& event)
{
    // Nothing to do if there aren't any selected items
    if(widgets.targetSetList->GetSelectedItemCount()== 0)
    {
        return;
    }

    eraseSelectedTargets();
}


void ScriptingPanel::saveTargetsPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets.scriptingWidget, wxT("Select targets file..."), wxT(""), wxT(""), wxT("*.tgt"), wxFD_SAVE);

    if(saveDialog.ShowModal() == wxID_OK)
    {
        wxString path = saveDialog.GetPath();
        std::ofstream out(std::string(path.mb_str()));

        if(out.good())
        {
            mpepc::MetricTargetSet targetSet(targets);
            targetSet.saveToFile(out);
        }
    }
}


void ScriptingPanel::loadTargetsPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.scriptingWidget, wxT("Select targets file..."), wxT(""), wxT(""), wxT("*.tgt"), wxFD_OPEN);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        wxString path = loadDialog.GetPath();
        std::ifstream in(std::string(path.mb_str()));

        if(in.good())
        {
            mpepc::MetricTargetSet targetSet(in);
            targets.clear();
            widgets.targetSetList->DeleteAllItems();
            for(auto& target : targetSet)
            {
                targets.push_back(target);
                addTargetToList(target);
            }
        }
    }
}


void ScriptingPanel::addTargetToScriptPressed(wxCommandEvent& event)
{
    auto selectedRows = getSelectedRows();

    for(auto row : selectedRows)
    {
        assert(row < targets.size());

        scriptTargets.push_back(targets[row]);
    }

    updateScriptList();
}


void ScriptingPanel::removeTargetFromScriptPressed(wxCommandEvent& event)
{
    long selection = widgets.scriptList->GetSelection();

    if(selection != wxNOT_FOUND)
    {
        removeTargetFromScript(selection);
    }
}


void ScriptingPanel::saveScriptPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets.scriptingWidget, wxT("Select planner script file..."), wxT(""), wxT(""), wxT("*.spt"), wxFD_SAVE);

    if(saveDialog.ShowModal() == wxID_OK)
    {
        // Create the tasks to be tossed into the script
        std::vector<mpepc::ScriptTask> tasks;
        for(auto& target : scriptTargets)
        {
            // if a task is created from a single pose it is always a pose task.
            tasks.push_back(mpepc::ScriptTask(target.pose, target.name, "Pose"));

            // TODO: to properly create script it needs to be done upstream from user input:
            // For example, when specifying DoorTask, it needs to have a button dedicated
            // to it so the user has to provide two consecutive poses that are the entry
            // and the exit.
            // Once we have that system installed then it should be those scripts are saved,
            // rather than this vector of named poses. The current system works only for
            // simple navigation scenario where no door and elevator is in the way. This is
            // what we are working on at the moment, so this is ok, for now.
        }

        // save script to a file
        wxString path = saveDialog.GetPath();
        mpepc::MetricPlannerScript script(std::string(path.mb_str()), tasks);
        script.save(std::string(path.mb_str()));
    }
}


void ScriptingPanel::loadScriptPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.scriptingWidget, wxT("Select planner script file..."), wxT(""), wxT(""), wxT("*.spt"), wxFD_OPEN);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        wxString path = loadDialog.GetPath();
        mpepc::MetricPlannerScript script(std::string(path.mb_str()));

        std::vector<mpepc::named_pose_t> loadedTargets;

        int targetCount = 0;
        for(auto& task : script)
        {
            std::vector<pose_t> poses = task.getTargets();
            std::vector<std::string> names = task.getTargetNames();

            for(std::size_t n = 0; n < poses.size(); ++n)
            {
                targetCount++;
                mpepc::named_pose_t namedTarget;
                namedTarget.name = names[n];
                namedTarget.pose = poses[n];

                loadedTargets.push_back(namedTarget);
            }
        }

        widgets.scriptingWidget->setCompletedTargets(loadedTargets);
    }
}

} // namespace ui
} // namespace vulcan
