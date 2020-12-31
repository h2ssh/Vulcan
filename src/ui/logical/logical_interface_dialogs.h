/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     logical_interface_dialogs.h
 * \author   Collin Johnson
 *
 * Declaration of LogicalDecisionDialog, LogicalGoalDialog, and LogicalTaskDialog.
 */

#ifndef UI_LOGICAL_LOGICAL_INTERFACE_DIALOGS_H
#define UI_LOGICAL_LOGICAL_INTERFACE_DIALOGS_H

#include "ui/logical/logical_interface.h"
#include "ui/logical/logical_interface_experiment.h"
#include <memory>

namespace vulcan
{
namespace planner
{
class GlobalTopoTarget;
class DecisionTarget;
}   // namespace planner

namespace ui
{

/**
 * LogicalDecisionDialog handles creation of a RelativePlaceTarget for commanding the
 * robot at the local topological layer of the HSSH. The decision dialog doesn't need
 * anything special. After the Go! button is pressed, the dialog will end and the created
 * target can be accessed via the getDecisionTarget() method.
 */
class LogicalDecisionDialog : public DecisionDialog
{
public:
    /**
     * Constructor for LogicalDecisionDialog.
     *
     * \param    parent          Parent window of the dialog
     */
    LogicalDecisionDialog(wxWindow* parent);

    /**
     * getDecisionTarget retrieves the target created using the dialog.
     */
    std::shared_ptr<planner::DecisionTarget> getDecisionTarget(void) const { return target; }

private:
    void handleDecisionButtonPressed(wxToggleButton* button);
    void createTarget(void);

    void leftDecisionButtonPressed(wxCommandEvent& event);
    void rightDecisionButtonPressed(wxCommandEvent& event);
    void forwardDecisionButtonPressed(wxCommandEvent& event);
    void backDecisionButtonPressed(wxCommandEvent& event);
    void goDecisionButtonPressed(wxCommandEvent& event);

    wxToggleButton* activeButton;

    std::shared_ptr<planner::DecisionTarget> target;

    DECLARE_EVENT_TABLE()
};

/**
 * LogicalGoalDialog handles creation of a GlobalPlaceTarget. On construction, the set of places to
 * choose from is provided. These places will be broken into groups based on the placesPerGroup parameter.
 * Once a place has been chosen, the getGoalTarget() method provides access to the goal.
 */
class LogicalGoalDialog : public GoalDialog
{
public:
    /**
     * Constructor for LogicalGoalDialog.
     *
     * \param    parent              Parent window of the dialog
     * \param    placeDescriptions   Mapping of description->placeId so the correct target can be selected
     * \param    map                 Map from which the goal is being selected
     */
    LogicalGoalDialog(wxWindow* parent,
                      const std::map<std::string, int>& placeDescriptions,
                      const hssh::TopologicalMap& map);

    /**
     * getGoalTarget retrieves the target created using the dialog.
     */
    std::shared_ptr<planner::GoalTarget> getGoalTarget(void) { return target; }

private:
    void populateGoalList(void);
    void createTarget(void);

    void goalListSelected(wxCommandEvent& event);
    void goGoalButtonPressed(wxCommandEvent& event);

    std::shared_ptr<planner::GoalTarget> target;
    const std::map<std::string, int>& descriptions;
    const hssh::TopologicalMap& map;

    DECLARE_EVENT_TABLE()
};

/**
 * LogicalTaskDialog is a dialog explaining the next task to be performed. The goal description
 * and
 */
class LogicalTaskDialog : public TaskDialog
{
public:
    /**
     * Constructor for LogicalTaskDialog.
     *
     * \param    parent              Parent window of the dialog
     * \param    goalDescription     Description of the goal to explain where the user needs to go next
     * \param    level               The command level to be used for getting to said place
     */
    LogicalTaskDialog(wxWindow* parent, const std::string& goalDescription, logical_task_level_t& level);

private:
    void okayButtonPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

/**
 * LogicalTaskCompleteDialog is the dialog that pops up when the user reaches the goal target. It just
 * tells them they made it. Then a new task dialog will popup afterwards.
 */
class LogicalTaskCompleteDialog : public TaskCompleteDialog
{
public:
    /**
     * Construct for LogicalTaskCompleteDialog.
     *
     * \param    parent          Parent window for the dialog
     */
    LogicalTaskCompleteDialog(wxWindow* parent);

private:
    void okayButtonPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

/**
 * LogicalExperimentCompleteDialog is the dialog that pops up when the final task has been completed. It just
 * tells the user thank you and then has a button to hit saying, 'Done'. After 'Done' is pressed, the user
 * interface just shuts down.
 */
class LogicalExperimentCompleteDialog : public ExperimentCompleteDialog
{
public:
    /**
     * Constructor for LogicalExperimentCompleteDialog.
     *
     * \param    parent          Parent window for the dialog
     */
    LogicalExperimentCompleteDialog(wxWindow* parent);

private:
    void doneButtonPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_LOGICAL_LOGICAL_INTERFACE_DIALOGS_H
