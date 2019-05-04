/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_dialogs.cpp
* \author   Collin Johnson
*
* Definition of LogicalDecisionDialog, LogicalGoalDialog, and LogicalTaskDialog.
*/

#include <ui/logical/logical_interface_dialogs.h>
#include <ui/logical/logical_interface_experiment.h>
#include <planner/decision/decision_target.h>
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LogicalDecisionDialog, wxDialog)
//     EVT_TOGGLEBUTTON(ID_FORWARD_DECISION_BUTTON, LogicalDecisionDialog::forwardDecisionButtonPressed)
//     EVT_TOGGLEBUTTON(ID_LEFT_DECISION_BUTTON,    LogicalDecisionDialog::leftDecisionButtonPressed)
//     EVT_TOGGLEBUTTON(ID_RIGHT_DECISION_BUTTON,   LogicalDecisionDialog::rightDecisionButtonPressed)
//     EVT_TOGGLEBUTTON(ID_BACK_DECISION_BUTTON,    LogicalDecisionDialog::backDecisionButtonPressed)
    EVT_BUTTON(ID_DECISION_GO_BUTTON, LogicalDecisionDialog::goDecisionButtonPressed)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(LogicalGoalDialog, wxDialog)
    EVT_LISTBOX(ID_GOAL_LIST,     LogicalGoalDialog::goalListSelected)
    EVT_BUTTON(ID_GOAL_GO_BUTTON, LogicalGoalDialog::goGoalButtonPressed)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(LogicalTaskDialog, wxDialog)
    EVT_BUTTON(ID_TASK_OK_BUTTON, LogicalTaskDialog::okayButtonPressed)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(LogicalTaskCompleteDialog, wxDialog)
    EVT_BUTTON(ID_TASK_COMPLETE_OK_BUTTON, LogicalTaskCompleteDialog::okayButtonPressed)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(LogicalExperimentCompleteDialog, wxDialog)
    EVT_BUTTON(ID_EXPERIMENT_DONE_BUTTON, LogicalExperimentCompleteDialog::doneButtonPressed)
END_EVENT_TABLE()


LogicalDecisionDialog::LogicalDecisionDialog(wxWindow* parent)
    : DecisionDialog(parent)
    , activeButton(0)
{
    forwardDecisionButton->SetValue(false);
    leftDecisionButton->SetValue(false);
    rightDecisionButton->SetValue(false);
    backDecisionButton->SetValue(false);
    decisionGoButton->Disable();
}

////////////////////////// LogicalDecisionDialog implementation /////////////////////////////////////////////?
void LogicalDecisionDialog::handleDecisionButtonPressed(wxToggleButton* button)
{
    // If the button has been set to true, then it can be the decision
    if(button->GetValue() == true)
    {
        // If there was already an active button, then it needs to be turned off to ensure
        // only a single button is active at any given time.
        if(activeButton && (activeButton != button))
        {
            activeButton->SetValue(false);
        }

        activeButton = button;
        decisionGoButton->Enable();
    }
    else // button was unselected
    {
        // If the button was set to false, that means it was previously true and therefore MUST be the activeButton
        // otherwise there is a bug. The activeButton going false means no selection is made, so the go button
        // needs to be disabled again

        assert(activeButton == button);

        activeButton = 0;
        decisionGoButton->Disable();
    }
}


void LogicalDecisionDialog::createTarget(void)
{
    // Use the active target to determine the direction of the relative target
    if(activeButton == forwardDecisionButton)
    {
        target = std::shared_ptr<planner::DecisionTarget>(new planner::RelativePlaceTarget(planner::DECISION_GO_STRAIGHT));
    }
    else if(activeButton == backDecisionButton)
    {
        target = std::shared_ptr<planner::DecisionTarget>(new planner::RelativePlaceTarget(planner::DECISION_GO_BACK));
    }
    else if(activeButton == leftDecisionButton)
    {
        target = std::shared_ptr<planner::DecisionTarget>(new planner::RelativePlaceTarget(planner::DECISION_TURN_LEFT));
    }
    else if(activeButton == rightDecisionButton)
    {
        target = std::shared_ptr<planner::DecisionTarget>(new planner::RelativePlaceTarget(planner::DECISION_TURN_RIGHT));
    }
    else
    {
        assert(activeButton);
    }
}


void LogicalDecisionDialog::leftDecisionButtonPressed(wxCommandEvent& event)
{
    handleDecisionButtonPressed(leftDecisionButton);
}


void LogicalDecisionDialog::rightDecisionButtonPressed(wxCommandEvent& event)
{
    handleDecisionButtonPressed(rightDecisionButton);
}


void LogicalDecisionDialog::forwardDecisionButtonPressed(wxCommandEvent& event)
{
    handleDecisionButtonPressed(forwardDecisionButton);
}


void LogicalDecisionDialog::backDecisionButtonPressed(wxCommandEvent& event)
{
    handleDecisionButtonPressed(backDecisionButton);
}


void LogicalDecisionDialog::goDecisionButtonPressed(wxCommandEvent& event)
{
    assert(activeButton);

    createTarget();

    EndModal(wxID_OK);
}

////////////////////////// LogicalGoalDialog implementation /////////////////////////////////////////////?
LogicalGoalDialog::LogicalGoalDialog(wxWindow* parent, const std::map<std::string, int>& placeDescriptions, const hssh::TopologicalMap& map)
    : GoalDialog(parent)
    , descriptions(placeDescriptions)
    , map(map)
{
    goalList->SetSelection(wxNOT_FOUND);
    goalGoButton->Disable();

    populateGoalList();
}


void LogicalGoalDialog::populateGoalList(void)
{
    goalList->Clear();

    for(auto descIt = descriptions.begin(), descEnd = descriptions.end(); descIt != descEnd; ++descIt)
    {
        goalList->Append(wxString::FromAscii(descIt->first.c_str()));
    }
}


void LogicalGoalDialog::createTarget(void)
{
    std::string description(goalList->GetStringSelection().mb_str());

    assert(descriptions.find(description) != descriptions.end());

    target = std::shared_ptr<planner::GoalTarget>(new planner::GoalTarget(0, map.getId(), map.getPlace(descriptions.find(description)->second)));
}


void LogicalGoalDialog::goalListSelected(wxCommandEvent& event)
{
    if(goalList->GetSelection() != wxNOT_FOUND)
    {
        goalGoButton->Enable();
    }
    else
    {
        goalGoButton->Disable();
    }
}


void LogicalGoalDialog::goGoalButtonPressed(wxCommandEvent& event)
{
    assert(goalList->GetSelection() != wxNOT_FOUND);

    createTarget();

    EndModal(wxID_OK);
}


LogicalTaskDialog::LogicalTaskDialog(wxWindow* parent, const std::string& goalDescription, logical_task_level_t& level)
    : TaskDialog(parent)
{
    goalIdText->SetLabel(wxString::FromAscii(goalDescription.c_str()));

    switch(level)
    {
    case LOGICAL_DECISION:
        interfaceText->SetLabel(wxT("Decision"));
        break;

    case LOGICAL_GOAL:
        interfaceText->SetLabel(wxT("Goal"));
        break;

    case LOGICAL_ANY:
        interfaceText->SetLabel(wxT("Any"));
        break;
    }
}


void LogicalTaskDialog::okayButtonPressed(wxCommandEvent& event)
{
    EndModal(wxID_OK);
}


LogicalTaskCompleteDialog::LogicalTaskCompleteDialog(wxWindow* parent)
    : TaskCompleteDialog(parent)
{
}


void LogicalTaskCompleteDialog::okayButtonPressed(wxCommandEvent& event)
{
    EndModal(wxID_OK);
}


LogicalExperimentCompleteDialog::LogicalExperimentCompleteDialog(wxWindow* parent)
    : ExperimentCompleteDialog(parent)
{
}


void LogicalExperimentCompleteDialog::doneButtonPressed(wxCommandEvent& event)
{
    EndModal(wxID_OK);
}

} // namespace ui
} // namespace vulcan
