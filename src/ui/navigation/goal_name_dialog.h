/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_name_dialog.h
* \author   Collin Johnson
*
* Declaration of GoalNameDialog.
*/

#ifndef UI_NAVIGATION_GOAL_NAME_DIALOG_H
#define UI_NAVIGATION_GOAL_NAME_DIALOG_H

#include "ui/navigation/navigation_interface.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

/**
* GoalNameDialog pops up a simple dialog with a text box so the user can enter the name of a goal.
*
* The dialog must be shown as a modal dialog box.
*/
class GoalNameDialog : public GoalNameDialogBase
{
public:

    GoalNameDialog(wxWindow* parent)
    : GoalNameDialogBase(parent)
    {
        nameOkButton->Disable();
    }

    /**
    * goalName retrieves the name provided by the user.
    */
    std::string goalName(void) const { return goalName_; }

private:

    std::string goalName_;

    // GoalNameDialogBase interface
    void goalNameTextEntered(wxCommandEvent& event) override
    {
        // As long as there is text in the name control, then okay can be pressed
        nameOkButton->Enable(!goalNameText->GetValue().IsEmpty());
    }

    void goalNameEnterPressed(wxCommandEvent& event) override
    {
        assert(IsModal());
        goalName_ = goalNameText->GetValue().ToStdString();

        std::cout << "Enter pressed. Name:" << goalName_ << '\n';

        // Once text has been entered, it is okay to close after enter is pressed
        if(!goalName_.empty())
        {
            EndModal(wxID_OK);
        }
    }

    void nameCancelPressed(wxCommandEvent& event) override
    {
        assert(IsModal());
        EndModal(wxID_CANCEL);
    }


    void nameOkayPressed(wxCommandEvent& event) override
    {
        assert(IsModal());
        goalName_ = goalNameText->GetValue().ToStdString();

        assert(!goalName_.empty());
        EndModal(wxID_OK);
    }
};

}
}

#endif // UI_NAVIGATION_GOAL_NAME_DIALOG_H
