///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "calibration_ui.h"

///////////////////////////////////////////////////////////////////////////

CalibrationFrame::CalibrationFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : UIMainFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* calibrationFrameSizer;
	calibrationFrameSizer = new wxBoxSizer( wxVERTICAL );

	calibrationNotebook = new wxAuiNotebook( this, ID_CALIBRATION_NOTEBOOK, wxDefaultPosition, wxDefaultSize, wxAUI_NB_DEFAULT_STYLE );
	wheelchairPanel = new wxPanel( calibrationNotebook, ID_WHEELCHAIR_PANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* wheelchairPanelSizer;
	wheelchairPanelSizer = new wxBoxSizer( wxHORIZONTAL );

	wxBoxSizer* wheelchairTestDisplaySizer;
	wheelchairTestDisplaySizer = new wxBoxSizer( wxVERTICAL );

	wxGridSizer* testNameSizer;
	testNameSizer = new wxGridSizer( 0, 2, 0, 0 );

	testNameLabelText = new wxStaticText( wheelchairPanel, wxID_ANY, wxT("Test:"), wxDefaultPosition, wxDefaultSize, 0 );
	testNameLabelText->Wrap( -1 );
	testNameLabelText->SetFont( wxFont( 20, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	testNameSizer->Add( testNameLabelText, 0, wxALIGN_RIGHT|wxALL, 5 );

	testNameText = new wxStaticText( wheelchairPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	testNameText->Wrap( -1 );
	testNameText->SetFont( wxFont( 20, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	testNameSizer->Add( testNameText, 0, wxALL, 5 );


	wheelchairTestDisplaySizer->Add( testNameSizer, 0, wxEXPAND, 5 );

	wxGridSizer* tasksCompletedSizer;
	tasksCompletedSizer = new wxGridSizer( 0, 2, 0, 0 );

	tasksCompletedLabelText = new wxStaticText( wheelchairPanel, wxID_ANY, wxT("# Tasks Completed:"), wxDefaultPosition, wxDefaultSize, 0 );
	tasksCompletedLabelText->Wrap( -1 );
	tasksCompletedLabelText->SetFont( wxFont( 16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	tasksCompletedSizer->Add( tasksCompletedLabelText, 0, wxALIGN_RIGHT|wxALL, 5 );

	tasksCompletedText = new wxStaticText( wheelchairPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	tasksCompletedText->Wrap( -1 );
	tasksCompletedText->SetFont( wxFont( 16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	tasksCompletedSizer->Add( tasksCompletedText, 0, wxALL, 5 );


	wheelchairTestDisplaySizer->Add( tasksCompletedSizer, 0, wxEXPAND, 5 );

	wxGridSizer* taskNotesSizer;
	taskNotesSizer = new wxGridSizer( 0, 2, 0, 0 );

	taskNotesLabelText = new wxStaticText( wheelchairPanel, wxID_ANY, wxT("Task Notes:"), wxDefaultPosition, wxDefaultSize, 0 );
	taskNotesLabelText->Wrap( -1 );
	taskNotesLabelText->SetFont( wxFont( 14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	taskNotesSizer->Add( taskNotesLabelText, 0, wxALIGN_RIGHT|wxALL, 5 );

	taskNotesText = new wxStaticText( wheelchairPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	taskNotesText->Wrap( -1 );
	taskNotesText->SetFont( wxFont( 14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString ) );

	taskNotesSizer->Add( taskNotesText, 0, wxALL, 5 );


	wheelchairTestDisplaySizer->Add( taskNotesSizer, 0, wxEXPAND, 5 );

	taskNotesLine = new wxStaticLine( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	wheelchairTestDisplaySizer->Add( taskNotesLine, 0, wxEXPAND | wxALL, 5 );

	wxFlexGridSizer* testResultsSizer;
	testResultsSizer = new wxFlexGridSizer( 2, 1, 0, 0 );
	testResultsSizer->SetFlexibleDirection( wxVERTICAL );
	testResultsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_ALL );

	testResultsLabelText = new wxStaticText( wheelchairPanel, wxID_ANY, wxT("Test Results"), wxDefaultPosition, wxDefaultSize, 0 );
	testResultsLabelText->Wrap( -1 );
	testResultsLabelText->SetFont( wxFont( 16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, true, wxEmptyString ) );

	testResultsSizer->Add( testResultsLabelText, 0, wxALIGN_CENTER|wxALL, 5 );

	testResultsGrid = new wxGrid( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );

	// Grid
	testResultsGrid->CreateGrid( 5, 5 );
	testResultsGrid->EnableEditing( false );
	testResultsGrid->EnableGridLines( true );
	testResultsGrid->EnableDragGridSize( false );
	testResultsGrid->SetMargins( 0, 0 );

	// Columns
	testResultsGrid->AutoSizeColumns();
	testResultsGrid->EnableDragColMove( false );
	testResultsGrid->EnableDragColSize( true );
	testResultsGrid->SetColLabelSize( 30 );
	testResultsGrid->SetColLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Rows
	testResultsGrid->EnableDragRowSize( true );
	testResultsGrid->SetRowLabelSize( 80 );
	testResultsGrid->SetRowLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Label Appearance

	// Cell Defaults
	testResultsGrid->SetDefaultCellAlignment( wxALIGN_LEFT, wxALIGN_TOP );
	testResultsSizer->Add( testResultsGrid, 0, wxALIGN_CENTER|wxALL, 5 );


	wheelchairTestDisplaySizer->Add( testResultsSizer, 0, wxALIGN_CENTER|wxEXPAND, 5 );

	testResultsLine = new wxStaticLine( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	wheelchairTestDisplaySizer->Add( testResultsLine, 0, wxEXPAND | wxALL, 5 );

	wxFlexGridSizer* systemDataSizer;
	systemDataSizer = new wxFlexGridSizer( 4, 1, 0, 0 );
	systemDataSizer->SetFlexibleDirection( wxVERTICAL );
	systemDataSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_ALL );

	systemDataLabel = new wxStaticText( wheelchairPanel, wxID_ANY, wxT("System Data"), wxDefaultPosition, wxDefaultSize, 0 );
	systemDataLabel->Wrap( -1 );
	systemDataLabel->SetFont( wxFont( 16, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, true, wxEmptyString ) );

	systemDataSizer->Add( systemDataLabel, 0, wxALIGN_CENTER|wxALL, 5 );

	odometryGrid = new wxGrid( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );

	// Grid
	odometryGrid->CreateGrid( 1, 5 );
	odometryGrid->EnableEditing( false );
	odometryGrid->EnableGridLines( true );
	odometryGrid->EnableDragGridSize( false );
	odometryGrid->SetMargins( 0, 0 );

	// Columns
	odometryGrid->EnableDragColMove( false );
	odometryGrid->EnableDragColSize( true );
	odometryGrid->SetColLabelValue( 0, wxT("x") );
	odometryGrid->SetColLabelValue( 1, wxT("y") );
	odometryGrid->SetColLabelValue( 2, wxT("theta") );
	odometryGrid->SetColLabelValue( 3, wxT("linear") );
	odometryGrid->SetColLabelValue( 4, wxT("angular") );
	odometryGrid->SetColLabelSize( 30 );
	odometryGrid->SetColLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Rows
	odometryGrid->EnableDragRowSize( true );
	odometryGrid->SetRowLabelValue( 0, wxT("Odometry:") );
	odometryGrid->SetRowLabelSize( 80 );
	odometryGrid->SetRowLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Label Appearance

	// Cell Defaults
	odometryGrid->SetDefaultCellAlignment( wxALIGN_LEFT, wxALIGN_TOP );
	systemDataSizer->Add( odometryGrid, 0, wxALIGN_CENTER|wxALL, 5 );

	imuGrid = new wxGrid( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );

	// Grid
	imuGrid->CreateGrid( 1, 6 );
	imuGrid->EnableEditing( false );
	imuGrid->EnableGridLines( true );
	imuGrid->EnableDragGridSize( false );
	imuGrid->SetMargins( 0, 0 );

	// Columns
	imuGrid->EnableDragColMove( false );
	imuGrid->EnableDragColSize( true );
	imuGrid->SetColLabelValue( 0, wxT("x-accel") );
	imuGrid->SetColLabelValue( 1, wxT("y-accel") );
	imuGrid->SetColLabelValue( 2, wxT("z-accel") );
	imuGrid->SetColLabelValue( 3, wxT("theta-vel") );
	imuGrid->SetColLabelValue( 4, wxT("rho-vel") );
	imuGrid->SetColLabelValue( 5, wxT("phi-vel") );
	imuGrid->SetColLabelSize( 30 );
	imuGrid->SetColLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Rows
	imuGrid->EnableDragRowSize( true );
	imuGrid->SetRowLabelValue( 0, wxT("IMU:") );
	imuGrid->SetRowLabelSize( 80 );
	imuGrid->SetRowLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Label Appearance

	// Cell Defaults
	imuGrid->SetDefaultCellAlignment( wxALIGN_LEFT, wxALIGN_TOP );
	systemDataSizer->Add( imuGrid, 0, wxALL, 5 );

	cmdVelocityGrid = new wxGrid( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );

	// Grid
	cmdVelocityGrid->CreateGrid( 1, 2 );
	cmdVelocityGrid->EnableEditing( false );
	cmdVelocityGrid->EnableGridLines( true );
	cmdVelocityGrid->EnableDragGridSize( false );
	cmdVelocityGrid->SetMargins( 0, 0 );

	// Columns
	cmdVelocityGrid->EnableDragColMove( false );
	cmdVelocityGrid->EnableDragColSize( true );
	cmdVelocityGrid->SetColLabelValue( 0, wxT("linear") );
	cmdVelocityGrid->SetColLabelValue( 1, wxT("angular") );
	cmdVelocityGrid->SetColLabelSize( 30 );
	cmdVelocityGrid->SetColLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Rows
	cmdVelocityGrid->EnableDragRowSize( true );
	cmdVelocityGrid->SetRowLabelValue( 0, wxT("Cmd Vel:") );
	cmdVelocityGrid->SetRowLabelSize( 80 );
	cmdVelocityGrid->SetRowLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Label Appearance

	// Cell Defaults
	cmdVelocityGrid->SetDefaultCellAlignment( wxALIGN_LEFT, wxALIGN_TOP );
	systemDataSizer->Add( cmdVelocityGrid, 0, wxALIGN_CENTER|wxALL, 5 );


	wheelchairTestDisplaySizer->Add( systemDataSizer, 1, wxALIGN_CENTER|wxEXPAND, 5 );


	wheelchairPanelSizer->Add( wheelchairTestDisplaySizer, 1, wxEXPAND, 5 );

	wheelchairTestLine = new wxStaticLine( wheelchairPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	wheelchairPanelSizer->Add( wheelchairTestLine, 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* wheelchairTestSelectionSizer;
	wheelchairTestSelectionSizer = new wxBoxSizer( wxVERTICAL );

	wxString wheelchairTestSelectionBoxChoices[] = { wxT("Stopping Distance"), wxT("Minimum Stopped Speed"), wxT("Minimum Rolling Speed"), wxT("Parameter Estimator") };
	int wheelchairTestSelectionBoxNChoices = sizeof( wheelchairTestSelectionBoxChoices ) / sizeof( wxString );
	wheelchairTestSelectionBox = new wxRadioBox( wheelchairPanel, ID_WHEELCHAIR_TEST_SELECTION_BOX, wxT("Wheelchair Tests:"), wxDefaultPosition, wxDefaultSize, wheelchairTestSelectionBoxNChoices, wheelchairTestSelectionBoxChoices, 1, wxRA_SPECIFY_COLS );
	wheelchairTestSelectionBox->SetSelection( 0 );
	wheelchairTestSelectionSizer->Add( wheelchairTestSelectionBox, 0, wxALL, 5 );

	wxBoxSizer* testSelectionSizer;
	testSelectionSizer = new wxBoxSizer( wxHORIZONTAL );

	runTestButton = new wxButton( wheelchairPanel, ID_RUN_TEST_BUTTON, wxT("Run"), wxDefaultPosition, wxDefaultSize, 0 );
	testSelectionSizer->Add( runTestButton, 0, wxALIGN_CENTER|wxALL, 5 );

	cancelTestButton = new wxButton( wheelchairPanel, ID_CANCEL_TEST_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	testSelectionSizer->Add( cancelTestButton, 0, wxALIGN_CENTER|wxALL, 5 );


	wheelchairTestSelectionSizer->Add( testSelectionSizer, 0, wxALIGN_CENTER|wxEXPAND, 5 );

	testNotesText = new wxStaticText( wheelchairPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	testNotesText->Wrap( -1 );
	wheelchairTestSelectionSizer->Add( testNotesText, 1, wxALL|wxEXPAND, 5 );


	wheelchairPanelSizer->Add( wheelchairTestSelectionSizer, 0, wxEXPAND, 5 );


	wheelchairPanel->SetSizer( wheelchairPanelSizer );
	wheelchairPanel->Layout();
	wheelchairPanelSizer->Fit( wheelchairPanel );
	calibrationNotebook->AddPage( wheelchairPanel, wxT("Wheelchair"), false, wxNullBitmap );
	playgroundPanel = new wxPanel( calibrationNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* playgroundPanelSizer;
	playgroundPanelSizer = new wxBoxSizer( wxHORIZONTAL );

	playgroundWidget = new PlaygroundDisplayWidget(playgroundPanel, ID_PLAYGROUND_WIDGET, wxDefaultPosition,wxDefaultSize);
	playgroundPanelSizer->Add( playgroundWidget, 1, wxALL|wxEXPAND, 5 );

	playgroundSettingsLine = new wxStaticLine( playgroundPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	playgroundPanelSizer->Add( playgroundSettingsLine, 0, wxEXPAND | wxALL, 5 );

	wxGridBagSizer* playgroundSetupSizer;
	playgroundSetupSizer = new wxGridBagSizer( 0, 0 );
	playgroundSetupSizer->SetFlexibleDirection( wxBOTH );
	playgroundSetupSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	controllerParametersLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("Controller Parameters:"), wxDefaultPosition, wxDefaultSize, 0 );
	controllerParametersLabel->Wrap( -1 );
	controllerParametersLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( controllerParametersLabel, wxGBPosition( 0, 0 ), wxGBSpan( 1, 3 ), wxALIGN_CENTER|wxALL, 5 );

	createControllerParametersButton = new wxButton( playgroundPanel, ID_CREATE_CONTROLLER_PARAMETERS_BUTTON, wxT("Create"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( createControllerParametersButton, wxGBPosition( 1, 0 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	loadControllerParametersButton = new wxButton( playgroundPanel, ID_LOAD_CONTROLLER_PARAMETERS_BUTTON, wxT("Load"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( loadControllerParametersButton, wxGBPosition( 1, 1 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	saveControllerParametersButton = new wxButton( playgroundPanel, ID_SAVE_CONTROLLER_PARAMETERS_BUTTON, wxT("Save"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( saveControllerParametersButton, wxGBPosition( 1, 2 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	playgroundBoundaryLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("Playground Boundary:"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundBoundaryLabel->Wrap( -1 );
	playgroundBoundaryLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( playgroundBoundaryLabel, wxGBPosition( 2, 0 ), wxGBSpan( 1, 3 ), wxALIGN_CENTER|wxALL, 5 );

	setPlaygroundBoundaryButton = new wxButton( playgroundPanel, ID_SET_PLAYGROUND_BOUNDARY_BUTTON, wxT("Set"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( setPlaygroundBoundaryButton, wxGBPosition( 3, 0 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	donePlaygroundBoundaryButton = new wxButton( playgroundPanel, ID_DONE_PLAYGROUND_BOUNDARY_BUTTON, wxT("Done"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( donePlaygroundBoundaryButton, wxGBPosition( 3, 1 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	clearPlaygroundBoundaryButton = new wxButton( playgroundPanel, ID_CLEAR_PLAYGROUND_BOUNDARY_BUTTON, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( clearPlaygroundBoundaryButton, wxGBPosition( 3, 2 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	targetRegionLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("Target Region:"), wxDefaultPosition, wxDefaultSize, 0 );
	targetRegionLabel->Wrap( -1 );
	targetRegionLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( targetRegionLabel, wxGBPosition( 4, 0 ), wxGBSpan( 1, 3 ), wxALIGN_CENTER|wxALL, 5 );

	setTargetRegionButton = new wxButton( playgroundPanel, ID_SET_TARGET_REGION_BUTTON, wxT("Set"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( setTargetRegionButton, wxGBPosition( 5, 0 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	doneTargetRegionButton = new wxButton( playgroundPanel, ID_DONE_TARGET_REGION_BUTTON, wxT("Done"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( doneTargetRegionButton, wxGBPosition( 5, 1 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	clearTargetRegionButton = new wxButton( playgroundPanel, ID_CLEAR_TARGET_REGION_BUTTON, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( clearTargetRegionButton, wxGBPosition( 5, 2 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	vMaxLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("v_max (m/s):"), wxDefaultPosition, wxDefaultSize, 0 );
	vMaxLabel->Wrap( -1 );
	vMaxLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( vMaxLabel, wxGBPosition( 6, 0 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	vMaxText = new wxTextCtrl( playgroundPanel, ID_VMAX_TEXT, wxT("1.0"), wxDefaultPosition, wxDefaultSize, wxTE_RIGHT );
	playgroundSetupSizer->Add( vMaxText, wxGBPosition( 6, 1 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER|wxALL, 5 );

	calcVMaxButton = new wxButton( playgroundPanel, ID_CALC_VMAX_BUTTON, wxT("Calc"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( calcVMaxButton, wxGBPosition( 6, 2 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	numTargetsLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("# Targets:"), wxDefaultPosition, wxDefaultSize, 0 );
	numTargetsLabel->Wrap( -1 );
	numTargetsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( numTargetsLabel, wxGBPosition( 7, 0 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	numTargetsText = new wxTextCtrl( playgroundPanel, ID_NUM_TARGETS_TEXT, wxT("100"), wxDefaultPosition, wxDefaultSize, wxTE_RIGHT );
	playgroundSetupSizer->Add( numTargetsText, wxGBPosition( 7, 1 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER|wxALL, 5 );

	timePerTargetLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("Time/Target:"), wxDefaultPosition, wxDefaultSize, 0 );
	timePerTargetLabel->Wrap( -1 );
	timePerTargetLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( timePerTargetLabel, wxGBPosition( 8, 0 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	timePerTargetText = new wxTextCtrl( playgroundPanel, ID_TIME_PER_TARGET_TEXT, wxT("10"), wxDefaultPosition, wxDefaultSize, wxTE_RIGHT );
	playgroundSetupSizer->Add( timePerTargetText, wxGBPosition( 8, 1 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER|wxALL, 5 );

	timeSecondsLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("seconds"), wxDefaultPosition, wxDefaultSize, 0 );
	timeSecondsLabel->Wrap( -1 );
	playgroundSetupSizer->Add( timeSecondsLabel, wxGBPosition( 8, 2 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER_VERTICAL|wxALIGN_LEFT|wxALL, 5 );

	logNameLabel = new wxStaticText( playgroundPanel, wxID_ANY, wxT("Log name:"), wxDefaultPosition, wxDefaultSize, 0 );
	logNameLabel->Wrap( -1 );
	logNameLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	playgroundSetupSizer->Add( logNameLabel, wxGBPosition( 9, 0 ), wxGBSpan( 1, 1 ), wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	logNameText = new wxTextCtrl( playgroundPanel, ID_LOG_NAME_TEXT, wxT("playground_data.log"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( logNameText, wxGBPosition( 9, 1 ), wxGBSpan( 1, 2 ), wxALL|wxEXPAND, 5 );

	startSessionButton = new wxButton( playgroundPanel, ID_START_SESSION_BUTTON, wxT("Start"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( startSessionButton, wxGBPosition( 10, 0 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	pauseSessionButton = new wxButton( playgroundPanel, ID_PAUSE_SESSION_BUTTON, wxT("Pause"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( pauseSessionButton, wxGBPosition( 10, 1 ), wxGBSpan( 1, 1 ), wxALL, 5 );

	stopSessionButton = new wxButton( playgroundPanel, ID_STOP_SESSION_BUTTON, wxT("Stop"), wxDefaultPosition, wxDefaultSize, 0 );
	playgroundSetupSizer->Add( stopSessionButton, wxGBPosition( 10, 2 ), wxGBSpan( 1, 1 ), wxALL, 5 );


	playgroundPanelSizer->Add( playgroundSetupSizer, 0, wxEXPAND, 5 );


	playgroundPanel->SetSizer( playgroundPanelSizer );
	playgroundPanel->Layout();
	playgroundPanelSizer->Fit( playgroundPanel );
	calibrationNotebook->AddPage( playgroundPanel, wxT("Playground"), true, wxNullBitmap );

	calibrationFrameSizer->Add( calibrationNotebook, 1, wxEXPAND | wxALL, 5 );


	this->SetSizer( calibrationFrameSizer );
	this->Layout();
	calibrationStatusBar = this->CreateStatusBar( 1, wxSTB_SIZEGRIP, wxID_ANY );

	this->Centre( wxBOTH );
}

CalibrationFrame::~CalibrationFrame()
{
}

WheelchairTestSetup::WheelchairTestSetup( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* testSetupSizer;
	testSetupSizer = new wxBoxSizer( wxVERTICAL );

	wxStaticBoxSizer* testNameSizer;
	testNameSizer = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Test Name") ), wxVERTICAL );

	testNameText = new wxTextCtrl( testNameSizer->GetStaticBox(), ID_TEST_NAME_TEXT, wxT("Test Name"), wxDefaultPosition, wxDefaultSize, 0 );
	testNameSizer->Add( testNameText, 0, wxALIGN_CENTER|wxALL|wxEXPAND, 5 );


	testSetupSizer->Add( testNameSizer, 0, wxEXPAND, 5 );

	wxStaticBoxSizer* testGridSizer;
	testGridSizer = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Test Parameters") ), wxVERTICAL );

	testSetupGrid = new wxGrid( testGridSizer->GetStaticBox(), ID_TEST_SETUP_GRID, wxDefaultPosition, wxDefaultSize, 0 );

	// Grid
	testSetupGrid->CreateGrid( 2, 4 );
	testSetupGrid->EnableEditing( true );
	testSetupGrid->EnableGridLines( true );
	testSetupGrid->EnableDragGridSize( false );
	testSetupGrid->SetMargins( 1, 1 );

	// Columns
	testSetupGrid->AutoSizeColumns();
	testSetupGrid->EnableDragColMove( false );
	testSetupGrid->EnableDragColSize( true );
	testSetupGrid->SetColLabelValue( 0, wxT("Start") );
	testSetupGrid->SetColLabelValue( 1, wxT("Step") );
	testSetupGrid->SetColLabelValue( 2, wxT("Stop") );
	testSetupGrid->SetColLabelValue( 3, wxT("Time Per Step") );
	testSetupGrid->SetColLabelSize( 30 );
	testSetupGrid->SetColLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Rows
	testSetupGrid->AutoSizeRows();
	testSetupGrid->EnableDragRowSize( true );
	testSetupGrid->SetRowLabelValue( 0, wxT("X") );
	testSetupGrid->SetRowLabelValue( 1, wxT("Y") );
	testSetupGrid->SetRowLabelSize( 80 );
	testSetupGrid->SetRowLabelAlignment( wxALIGN_CENTER, wxALIGN_CENTER );

	// Label Appearance

	// Cell Defaults
	testSetupGrid->SetDefaultCellAlignment( wxALIGN_RIGHT, wxALIGN_CENTER );
	testGridSizer->Add( testSetupGrid, 0, wxALIGN_CENTER|wxALL|wxEXPAND, 5 );


	testSetupSizer->Add( testGridSizer, 0, wxEXPAND, 5 );

	wxString testCommandModeRadioBoxChoices[] = { wxT("Step"), wxT("Ramp") };
	int testCommandModeRadioBoxNChoices = sizeof( testCommandModeRadioBoxChoices ) / sizeof( wxString );
	testCommandModeRadioBox = new wxRadioBox( this, ID_TEST_COMMAND_MODE_RADIO_BOX, wxT("Command Mode"), wxDefaultPosition, wxDefaultSize, testCommandModeRadioBoxNChoices, testCommandModeRadioBoxChoices, 1, wxRA_SPECIFY_COLS );
	testCommandModeRadioBox->SetSelection( 0 );
	testSetupSizer->Add( testCommandModeRadioBox, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	wxBoxSizer* rampDurationSizer;
	rampDurationSizer = new wxBoxSizer( wxHORIZONTAL );

	rampDurationLabel = new wxStaticText( this, ID_RAMP_DURATION_LABEL, wxT("Ramp Duration (change/sec):"), wxDefaultPosition, wxDefaultSize, 0 );
	rampDurationLabel->Wrap( -1 );
	rampDurationSizer->Add( rampDurationLabel, 0, wxALIGN_CENTER|wxALL, 5 );

	rampDurationText = new wxTextCtrl( this, ID_RAMP_DURATION_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_RIGHT );
	rampDurationSizer->Add( rampDurationText, 0, wxALIGN_CENTER|wxALL, 5 );


	testSetupSizer->Add( rampDurationSizer, 1, wxEXPAND, 5 );

	wxGridSizer* saveCancelSizer;
	saveCancelSizer = new wxGridSizer( 1, 2, 0, 0 );

	saveTestButton = new wxButton( this, ID_SAVE_TEST_BUTTON, wxT("Save"), wxDefaultPosition, wxDefaultSize, 0 );
	saveCancelSizer->Add( saveTestButton, 0, wxALIGN_RIGHT|wxALL, 5 );

	cancelTestButton = new wxButton( this, ID_CANCEL_TEST_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	saveCancelSizer->Add( cancelTestButton, 0, wxALIGN_LEFT|wxALL, 5 );


	testSetupSizer->Add( saveCancelSizer, 1, wxEXPAND, 5 );


	this->SetSizer( testSetupSizer );
	this->Layout();

	this->Centre( wxBOTH );
}

WheelchairTestSetup::~WheelchairTestSetup()
{
}

ControlLawParameters::ControlLawParameters( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxGridSizer* controlLawSizer;
	controlLawSizer = new wxGridSizer( 10, 2, 0, 0 );

	k1Label = new wxStaticText( this, wxID_ANY, wxT("k1:"), wxDefaultPosition, wxDefaultSize, 0 );
	k1Label->Wrap( -1 );
	k1Label->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( k1Label, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	k1Text = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( k1Text, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	k2Label = new wxStaticText( this, wxID_ANY, wxT("k2:"), wxDefaultPosition, wxDefaultSize, 0 );
	k2Label->Wrap( -1 );
	k2Label->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( k2Label, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	k2Text = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( k2Text, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	maxLinearLabel = new wxStaticText( this, wxID_ANY, wxT("Max Linear Vel:"), wxDefaultPosition, wxDefaultSize, 0 );
	maxLinearLabel->Wrap( -1 );
	maxLinearLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( maxLinearLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	maxLinearVelText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( maxLinearVelText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	maxAngularVelLabel = new wxStaticText( this, wxID_ANY, wxT("Max Angular Vel:"), wxDefaultPosition, wxDefaultSize, 0 );
	maxAngularVelLabel->Wrap( -1 );
	maxAngularVelLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( maxAngularVelLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	maxAngularVelText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( maxAngularVelText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	angVelAtTargetLabel = new wxStaticText( this, wxID_ANY, wxT("Angular Vel at Target:"), wxDefaultPosition, wxDefaultSize, 0 );
	angVelAtTargetLabel->Wrap( -1 );
	angVelAtTargetLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( angVelAtTargetLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	angVelAtTargetText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( angVelAtTargetText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	slowdownRadiusLabel = new wxStaticText( this, wxID_ANY, wxT("Slowdown Radius:"), wxDefaultPosition, wxDefaultSize, 0 );
	slowdownRadiusLabel->Wrap( -1 );
	slowdownRadiusLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( slowdownRadiusLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	slowdownRadiusText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( slowdownRadiusText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	convergenceRadius = new wxStaticText( this, wxID_ANY, wxT("Convergence Radius:"), wxDefaultPosition, wxDefaultSize, 0 );
	convergenceRadius->Wrap( -1 );
	convergenceRadius->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( convergenceRadius, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	convergenceRadiusText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( convergenceRadiusText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	convergenceAngleLabel = new wxStaticText( this, wxID_ANY, wxT("Convergence Angle:"), wxDefaultPosition, wxDefaultSize, 0 );
	convergenceAngleLabel->Wrap( -1 );
	convergenceAngleLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( convergenceAngleLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	convergenceAngleText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( convergenceAngleText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	betaLabel = new wxStaticText( this, wxID_ANY, wxT("beta:"), wxDefaultPosition, wxDefaultSize, 0 );
	betaLabel->Wrap( -1 );
	betaLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( betaLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	betaText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( betaText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	lambdaLabel = new wxStaticText( this, wxID_ANY, wxT("lambda:"), wxDefaultPosition, wxDefaultSize, 0 );
	lambdaLabel->Wrap( -1 );
	lambdaLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	controlLawSizer->Add( lambdaLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	lambdaText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( lambdaText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	controlLawDialogOkayButton = new wxButton( this, ID_CONTROL_LAW_DIALOG_OKAY_BUTTON, wxT("Okay"), wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( controlLawDialogOkayButton, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	controlLawDialogCancelButton = new wxButton( this, ID_CONTROL_LAW_DIALOG_CANCEL_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	controlLawSizer->Add( controlLawDialogCancelButton, 0, wxALL, 5 );


	this->SetSizer( controlLawSizer );
	this->Layout();

	this->Centre( wxBOTH );
}

ControlLawParameters::~ControlLawParameters()
{
}
