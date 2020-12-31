///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "logical_interface.h"

///////////////////////////////////////////////////////////////////////////
using namespace vulcan::ui;

LogicalFrame::LogicalFrame(wxWindow* parent,
                           wxWindowID id,
                           const wxString& title,
                           const wxPoint& pos,
                           const wxSize& size,
                           long style)
: UIMainFrame(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* logicalFrameSizer;
    logicalFrameSizer = new wxBoxSizer(wxVERTICAL);

    plannerWidget = new GoalPlannerDisplayWidget(this, ID_GOAL_PLANNER_WIDGET, wxDefaultPosition, wxDefaultSize);
    logicalFrameSizer->Add(plannerWidget, 1, wxALL | wxEXPAND, 5);

    wxBoxSizer* allButtonsSizer;
    allButtonsSizer = new wxBoxSizer(wxVERTICAL);

    wxGridSizer* commandSizer;
    commandSizer = new wxGridSizer(1, 2, 0, 0);

    decisionButton = new wxButton(this, ID_DECISION_BUTTON, wxT("Decision"), wxDefaultPosition, wxDefaultSize, 0);
    decisionButton->SetFont(
      wxFont(30, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    commandSizer->Add(decisionButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    goalButton = new wxButton(this, ID_GOAL_BUTTON, wxT("Goal"), wxDefaultPosition, wxDefaultSize, 0);
    goalButton->SetFont(wxFont(30, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    commandSizer->Add(goalButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    allButtonsSizer->Add(commandSizer, 0, wxALL | wxEXPAND, 5);

    stopButton = new wxButton(this, ID_STOP_BUTTON, wxT("STOP!"), wxDefaultPosition, wxDefaultSize, 0);
    stopButton->SetFont(wxFont(45, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    allButtonsSizer->Add(stopButton, 0, wxALL | wxEXPAND, 5);


    logicalFrameSizer->Add(allButtonsSizer, 0, wxEXPAND, 5);


    this->SetSizer(logicalFrameSizer);
    this->Layout();

    this->Centre(wxBOTH);
}

LogicalFrame::~LogicalFrame()
{
}

DecisionDialog::DecisionDialog(wxWindow* parent,
                               wxWindowID id,
                               const wxString& title,
                               const wxPoint& pos,
                               const wxSize& size,
                               long style)
: wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxGridSizer* decisionButtonSizer;
    decisionButtonSizer = new wxGridSizer(4, 1, 0, 0);

    forwardDecisionButton =
      new wxToggleButton(this, ID_FORWARD_DECISION_BUTTON, wxT("Forward"), wxDefaultPosition, wxDefaultSize, 0);
    forwardDecisionButton->SetFont(
      wxFont(30, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    decisionButtonSizer->Add(forwardDecisionButton, 0, wxALIGN_CENTER | wxALL, 5);

    wxGridSizer* turnCommandSizer;
    turnCommandSizer = new wxGridSizer(1, 2, 0, 0);

    leftDecisionButton =
      new wxToggleButton(this, ID_LEFT_DECISION_BUTTON, wxT("Left"), wxDefaultPosition, wxDefaultSize, 0);
    leftDecisionButton->SetFont(
      wxFont(30, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    turnCommandSizer->Add(leftDecisionButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    rightDecisionButton =
      new wxToggleButton(this, ID_RIGHT_DECISION_BUTTON, wxT("Right"), wxDefaultPosition, wxDefaultSize, 0);
    rightDecisionButton->SetFont(
      wxFont(30, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    turnCommandSizer->Add(rightDecisionButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    decisionButtonSizer->Add(turnCommandSizer, 1, wxEXPAND, 5);

    backDecisionButton =
      new wxToggleButton(this, ID_BACK_DECISION_BUTTON, wxT("  Back  "), wxDefaultPosition, wxDefaultSize, 0);
    backDecisionButton->SetFont(
      wxFont(30, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    decisionButtonSizer->Add(backDecisionButton, 0, wxALIGN_CENTER | wxALL, 5);

    goalSeparator = new wxStaticLine(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL);
    decisionButtonSizer->Add(goalSeparator, 0, wxEXPAND | wxALL, 5);

    decisionGoButton = new wxButton(this, ID_DECISION_GO_BUTTON, wxT("GO!"), wxDefaultPosition, wxDefaultSize, 0);
    decisionGoButton->SetFont(
      wxFont(30, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    decisionButtonSizer->Add(decisionGoButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    this->SetSizer(decisionButtonSizer);
    this->Layout();

    this->Centre(wxBOTH);
}

DecisionDialog::~DecisionDialog()
{
}

GoalDialog::GoalDialog(wxWindow* parent,
                       wxWindowID id,
                       const wxString& title,
                       const wxPoint& pos,
                       const wxSize& size,
                       long style)
: wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* goalDialogSizer;
    goalDialogSizer = new wxBoxSizer(wxVERTICAL);

    wxBoxSizer* goalSelectionSizer;
    goalSelectionSizer = new wxBoxSizer(wxHORIZONTAL);

    goalList =
      new wxListBox(this, ID_GOAL_LIST, wxDefaultPosition, wxDefaultSize, 0, NULL, wxLB_NEEDED_SB | wxLB_SINGLE);
    goalList->Append(wxT("A-F"));
    goalList->Append(wxT("G-K"));
    goalList->Append(wxT("L-P"));
    goalList->Append(wxT("Q-T"));
    goalList->Append(wxT("U-V"));
    goalList->SetFont(wxFont(20, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    goalSelectionSizer->Add(goalList, 1, wxALL | wxEXPAND, 5);


    goalDialogSizer->Add(goalSelectionSizer, 1, wxEXPAND, 5);

    goalGoButton = new wxButton(this, ID_GOAL_GO_BUTTON, wxT("Go!"), wxDefaultPosition, wxDefaultSize, 0);
    goalGoButton->SetFont(wxFont(30, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("sans")));

    goalDialogSizer->Add(goalGoButton, 0, wxALL | wxEXPAND, 5);


    this->SetSizer(goalDialogSizer);
    this->Layout();

    this->Centre(wxBOTH);
}

GoalDialog::~GoalDialog()
{
}

TaskDialog::TaskDialog(wxWindow* parent,
                       wxWindowID id,
                       const wxString& title,
                       const wxPoint& pos,
                       const wxSize& size,
                       long style)
: wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxFlexGridSizer* taskAssignmentSizer;
    taskAssignmentSizer = new wxFlexGridSizer(2, 2, 0, 0);
    taskAssignmentSizer->SetFlexibleDirection(wxHORIZONTAL);
    taskAssignmentSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    taskGoalText = new wxStaticText(this, wxID_ANY, wxT("Move to place: "), wxDefaultPosition, wxDefaultSize, 0);
    taskGoalText->Wrap(-1);
    taskGoalText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskAssignmentSizer->Add(taskGoalText, 0, wxALL, 5);

    goalIdText = new wxStaticText(this, wxID_ANY, wxT("A"), wxDefaultPosition, wxDefaultSize, 0);
    goalIdText->Wrap(-1);
    goalIdText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskAssignmentSizer->Add(goalIdText, 0, wxALL, 5);

    interfaceToUseText =
      new wxStaticText(this, wxID_ANY, wxT("Interface to use:"), wxDefaultPosition, wxDefaultSize, 0);
    interfaceToUseText->Wrap(-1);
    interfaceToUseText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskAssignmentSizer->Add(interfaceToUseText, 0, wxALL, 5);

    interfaceText = new wxStaticText(this, wxID_ANY, wxT("Decision"), wxDefaultPosition, wxDefaultSize, 0);
    interfaceText->Wrap(-1);
    interfaceText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskAssignmentSizer->Add(interfaceText, 0, wxALL, 5);

    taskOkButton = new wxButton(this, ID_TASK_OK_BUTTON, wxT("Ok"), wxDefaultPosition, wxDefaultSize, 0);
    taskOkButton->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskAssignmentSizer->Add(taskOkButton, 0, wxALIGN_RIGHT | wxALL, 5);


    this->SetSizer(taskAssignmentSizer);
    this->Layout();

    this->Centre(wxBOTH);
}

TaskDialog::~TaskDialog()
{
}

TaskCompleteDialog::TaskCompleteDialog(wxWindow* parent,
                                       wxWindowID id,
                                       const wxString& title,
                                       const wxPoint& pos,
                                       const wxSize& size,
                                       long style)
: wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* taskCompleteSizer;
    taskCompleteSizer = new wxBoxSizer(wxVERTICAL);

    taskCompleteText = new wxStaticText(this,
                                        wxID_ANY,
                                        wxT("Great! You reached the goal.\n\nHit 'Ok' for your next task."),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        wxALIGN_CENTER_HORIZONTAL);
    taskCompleteText->Wrap(-1);
    taskCompleteText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskCompleteSizer->Add(taskCompleteText, 0, wxALL, 5);

    completeHiddenText = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    completeHiddenText->Wrap(-1);
    completeHiddenText->SetFont(
      wxFont(20, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskCompleteSizer->Add(completeHiddenText, 0, wxALL, 5);

    taskCompleteOkButton =
      new wxButton(this, ID_TASK_COMPLETE_OK_BUTTON, wxT("Ok"), wxDefaultPosition, wxDefaultSize, 0);
    taskCompleteOkButton->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    taskCompleteSizer->Add(taskCompleteOkButton, 0, wxALIGN_CENTER | wxALL, 5);


    this->SetSizer(taskCompleteSizer);
    this->Layout();
    taskCompleteSizer->Fit(this);

    this->Centre(wxBOTH);
}

TaskCompleteDialog::~TaskCompleteDialog()
{
}

ExperimentCompleteDialog::ExperimentCompleteDialog(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxString& title,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long style)
: wxDialog(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* experimentCompleteSizer;
    experimentCompleteSizer = new wxBoxSizer(wxVERTICAL);

    experimentCompleteText = new wxStaticText(this,
                                              wxID_ANY,
                                              wxT("The experiment is finished.\n\nThank you!"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              wxALIGN_CENTER_HORIZONTAL);
    experimentCompleteText->Wrap(-1);
    experimentCompleteText->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    experimentCompleteSizer->Add(experimentCompleteText, 0, wxALL, 5);

    experimentHiddenText = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
    experimentHiddenText->Wrap(-1);
    experimentHiddenText->SetFont(
      wxFont(20, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    experimentCompleteSizer->Add(experimentHiddenText, 0, wxALL, 5);

    experimentDoneButton =
      new wxButton(this, ID_EXPERIMENT_DONE_BUTTON, wxT("Done"), wxDefaultPosition, wxDefaultSize, 0);
    experimentDoneButton->SetFont(
      wxFont(25, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    experimentCompleteSizer->Add(experimentDoneButton, 0, wxALIGN_CENTER | wxALL, 5);


    this->SetSizer(experimentCompleteSizer);
    this->Layout();
    experimentCompleteSizer->Fit(this);

    this->Centre(wxBOTH);
}

ExperimentCompleteDialog::~ExperimentCompleteDialog()
{
}
