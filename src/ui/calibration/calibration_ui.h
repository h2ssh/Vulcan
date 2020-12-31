///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include "ui/common/ui_main_frame.h"
#include <ui/calibration/playground_display_widget.h>
#include <wx/artprov.h>
#include <wx/aui/auibook.h>
#include <wx/bitmap.h>
#include <wx/button.h>
#include <wx/colour.h>
#include <wx/dialog.h>
#include <wx/font.h>
#include <wx/frame.h>
#include <wx/gbsizer.h>
#include <wx/gdicmn.h>
#include <wx/grid.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/statusbr.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/xrc/xmlres.h>

///////////////////////////////////////////////////////////////////////////

#define ID_CALIBRATION_NOTEBOOK 1000
#define ID_WHEELCHAIR_PANEL 1001
#define ID_WHEELCHAIR_TEST_SELECTION_BOX 1002
#define ID_RUN_TEST_BUTTON 1003
#define ID_CANCEL_TEST_BUTTON 1004
#define ID_PLAYGROUND_WIDGET 1005
#define ID_CREATE_CONTROLLER_PARAMETERS_BUTTON 1006
#define ID_LOAD_CONTROLLER_PARAMETERS_BUTTON 1007
#define ID_SAVE_CONTROLLER_PARAMETERS_BUTTON 1008
#define ID_SET_PLAYGROUND_BOUNDARY_BUTTON 1009
#define ID_DONE_PLAYGROUND_BOUNDARY_BUTTON 1010
#define ID_CLEAR_PLAYGROUND_BOUNDARY_BUTTON 1011
#define ID_SET_TARGET_REGION_BUTTON 1012
#define ID_DONE_TARGET_REGION_BUTTON 1013
#define ID_CLEAR_TARGET_REGION_BUTTON 1014
#define ID_VMAX_TEXT 1015
#define ID_CALC_VMAX_BUTTON 1016
#define ID_NUM_TARGETS_TEXT 1017
#define ID_TIME_PER_TARGET_TEXT 1018
#define ID_LOG_NAME_TEXT 1019
#define ID_START_SESSION_BUTTON 1020
#define ID_PAUSE_SESSION_BUTTON 1021
#define ID_STOP_SESSION_BUTTON 1022
#define ID_TEST_NAME_TEXT 1023
#define ID_TEST_SETUP_GRID 1024
#define ID_TEST_COMMAND_MODE_RADIO_BOX 1025
#define ID_RAMP_DURATION_LABEL 1026
#define ID_RAMP_DURATION_TEXT 1027
#define ID_SAVE_TEST_BUTTON 1028
#define ID_CONTROL_LAW_DIALOG_OKAY_BUTTON 1029
#define ID_CONTROL_LAW_DIALOG_CANCEL_BUTTON 1030

///////////////////////////////////////////////////////////////////////////////
/// Class CalibrationFrame
///////////////////////////////////////////////////////////////////////////////
class CalibrationFrame : public UIMainFrame
{
private:
protected:
    wxAuiNotebook* calibrationNotebook;
    wxPanel* wheelchairPanel;
    wxStaticText* testNameLabelText;
    wxStaticText* testNameText;
    wxStaticText* tasksCompletedLabelText;
    wxStaticText* tasksCompletedText;
    wxStaticText* taskNotesLabelText;
    wxStaticText* taskNotesText;
    wxStaticLine* taskNotesLine;
    wxStaticText* testResultsLabelText;
    wxGrid* testResultsGrid;
    wxStaticLine* testResultsLine;
    wxStaticText* systemDataLabel;
    wxGrid* odometryGrid;
    wxGrid* imuGrid;
    wxGrid* cmdVelocityGrid;
    wxStaticLine* wheelchairTestLine;
    wxRadioBox* wheelchairTestSelectionBox;
    wxButton* runTestButton;
    wxButton* cancelTestButton;
    wxStaticText* testNotesText;
    wxPanel* playgroundPanel;
    PlaygroundDisplayWidget* playgroundWidget;
    wxStaticLine* playgroundSettingsLine;
    wxStaticText* controllerParametersLabel;
    wxButton* createControllerParametersButton;
    wxButton* loadControllerParametersButton;
    wxButton* saveControllerParametersButton;
    wxStaticText* playgroundBoundaryLabel;
    wxButton* setPlaygroundBoundaryButton;
    wxButton* donePlaygroundBoundaryButton;
    wxButton* clearPlaygroundBoundaryButton;
    wxStaticText* targetRegionLabel;
    wxButton* setTargetRegionButton;
    wxButton* doneTargetRegionButton;
    wxButton* clearTargetRegionButton;
    wxStaticText* vMaxLabel;
    wxTextCtrl* vMaxText;
    wxButton* calcVMaxButton;
    wxStaticText* numTargetsLabel;
    wxTextCtrl* numTargetsText;
    wxStaticText* timePerTargetLabel;
    wxTextCtrl* timePerTargetText;
    wxStaticText* timeSecondsLabel;
    wxStaticText* logNameLabel;
    wxTextCtrl* logNameText;
    wxButton* startSessionButton;
    wxButton* pauseSessionButton;
    wxButton* stopSessionButton;
    wxStatusBar* calibrationStatusBar;

public:
    CalibrationFrame(wxWindow* parent,
                     wxWindowID id = wxID_ANY,
                     const wxString& title = wxT("Robot Calibration UI"),
                     const wxPoint& pos = wxDefaultPosition,
                     const wxSize& size = wxSize(851, 639),
                     long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);

    ~CalibrationFrame();
};

///////////////////////////////////////////////////////////////////////////////
/// Class WheelchairTestSetup
///////////////////////////////////////////////////////////////////////////////
class WheelchairTestSetup : public wxDialog
{
private:
protected:
    wxTextCtrl* testNameText;
    wxGrid* testSetupGrid;
    wxRadioBox* testCommandModeRadioBox;
    wxStaticText* rampDurationLabel;
    wxTextCtrl* rampDurationText;
    wxButton* saveTestButton;
    wxButton* cancelTestButton;

public:
    WheelchairTestSetup(wxWindow* parent,
                        wxWindowID id = wxID_ANY,
                        const wxString& title = wxT("Wheelchair Test Setup"),
                        const wxPoint& pos = wxDefaultPosition,
                        const wxSize& size = wxSize(374, 350),
                        long style = wxCAPTION | wxCLOSE_BOX);
    ~WheelchairTestSetup();
};

///////////////////////////////////////////////////////////////////////////////
/// Class ControlLawParameters
///////////////////////////////////////////////////////////////////////////////
class ControlLawParameters : public wxDialog
{
private:
protected:
    wxStaticText* k1Label;
    wxTextCtrl* k1Text;
    wxStaticText* k2Label;
    wxTextCtrl* k2Text;
    wxStaticText* maxLinearLabel;
    wxTextCtrl* maxLinearVelText;
    wxStaticText* maxAngularVelLabel;
    wxTextCtrl* maxAngularVelText;
    wxStaticText* angVelAtTargetLabel;
    wxTextCtrl* angVelAtTargetText;
    wxStaticText* slowdownRadiusLabel;
    wxTextCtrl* slowdownRadiusText;
    wxStaticText* convergenceRadius;
    wxTextCtrl* convergenceRadiusText;
    wxStaticText* convergenceAngleLabel;
    wxTextCtrl* convergenceAngleText;
    wxStaticText* betaLabel;
    wxTextCtrl* betaText;
    wxStaticText* lambdaLabel;
    wxTextCtrl* lambdaText;
    wxButton* controlLawDialogOkayButton;
    wxButton* controlLawDialogCancelButton;

public:
    ControlLawParameters(wxWindow* parent,
                         wxWindowID id = wxID_ANY,
                         const wxString& title = wxT("Control Law Parameters"),
                         const wxPoint& pos = wxDefaultPosition,
                         const wxSize& size = wxSize(300, 500),
                         long style = wxDEFAULT_DIALOG_STYLE);
    ~ControlLawParameters();
};
