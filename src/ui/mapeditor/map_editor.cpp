///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "ui/mapeditor/global_topo_editor_widget.h"
#include "ui/mapeditor/local_topo_editor_widget.h"
#include "ui/mapeditor/metric_editor_widget.h"

#include "map_editor.h"

///////////////////////////////////////////////////////////////////////////
using namespace vulcan::ui;

EditorFrame::EditorFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : UIMainFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* mapEditorSizer;
	mapEditorSizer = new wxBoxSizer( wxVERTICAL );

	mapEditorNotebook = new wxAuiNotebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_NB_DEFAULT_STYLE );
	localMetricNotebookPanel = new wxPanel( mapEditorNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* localMetricPanelSizer;
	localMetricPanelSizer = new wxBoxSizer( wxHORIZONTAL );

	metricEditorWidget = new MetricEditorWidget(localMetricNotebookPanel, ID_METRIC_EDITOR_WIDGET, wxDefaultPosition,wxDefaultSize);
	localMetricPanelSizer->Add( metricEditorWidget, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* metricEditorOptionsSizer;
	metricEditorOptionsSizer = new wxBoxSizer( wxVERTICAL );

	wxStaticBoxSizer* metricMapLoadSizer;
	metricMapLoadSizer = new wxStaticBoxSizer( new wxStaticBox( localMetricNotebookPanel, wxID_ANY, wxT("Map Options") ), wxVERTICAL );

	loadMetricFromFileButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_LOAD_METRIC_FROM_FILE_BUTTON, wxT("Load From File"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( loadMetricFromFileButton, 0, wxALIGN_CENTER|wxALL|wxEXPAND, 5 );

	captureMetricFromLCMButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_CAPTURE_METRIC_FROM_LCM_BUTTON, wxT("Capture From LCM"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( captureMetricFromLCMButton, 0, wxALL|wxEXPAND, 5 );

	createMetricButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_CREATE_METRIC_BUTTON, wxT("Create Empty"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( createMetricButton, 0, wxALIGN_CENTER|wxALL|wxEXPAND, 5 );

	saveToFileMetricButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_SAVE_TO_FILE_METRIC_BUTTON, wxT("Save To File"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( saveToFileMetricButton, 0, wxALL|wxEXPAND, 5 );

	sendViaLCMMetricButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_SEND_VIA_LCM_METRIC_BUTTON, wxT("Send Via LCM"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( sendViaLCMMetricButton, 0, wxALIGN_CENTER|wxALL|wxEXPAND, 5 );

	importFromImageButton = new wxButton( metricMapLoadSizer->GetStaticBox(), ID_IMPORT_FROM_IMAGE_BUTTON, wxT("Import From Image"), wxDefaultPosition, wxDefaultSize, 0 );
	metricMapLoadSizer->Add( importFromImageButton, 0, wxALL|wxEXPAND, 5 );


	metricEditorOptionsSizer->Add( metricMapLoadSizer, 0, wxEXPAND, 5 );

	wxStaticBoxSizer* cellEditingSizer;
	cellEditingSizer = new wxStaticBoxSizer( new wxStaticBox( localMetricNotebookPanel, wxID_ANY, wxT("Cell Editing Options") ), wxVERTICAL );

	wxString editCellTypeRadioBoxChoices[] = { wxT("Hazard"), wxT("Quasi-Static"), wxT("Occupied"), wxT("Free"), wxT("Unobserved") };
	int editCellTypeRadioBoxNChoices = sizeof( editCellTypeRadioBoxChoices ) / sizeof( wxString );
	editCellTypeRadioBox = new wxRadioBox( cellEditingSizer->GetStaticBox(), ID_EDIT_CELL_TYPE_RADIO_BOX, wxT("Cell Type"), wxDefaultPosition, wxDefaultSize, editCellTypeRadioBoxNChoices, editCellTypeRadioBoxChoices, 1, wxRA_SPECIFY_COLS );
	editCellTypeRadioBox->SetSelection( 0 );
	cellEditingSizer->Add( editCellTypeRadioBox, 0, wxALL|wxEXPAND, 5 );

	cellEditModeButton = new wxToggleButton( cellEditingSizer->GetStaticBox(), ID_CELL_EDIT_MODE_BUTTON, wxT("Edit Cells"), wxDefaultPosition, wxDefaultSize, 0 );
	cellEditingSizer->Add( cellEditModeButton, 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* lineAndFloodButtonSizer;
	lineAndFloodButtonSizer = new wxBoxSizer( wxHORIZONTAL );

	lineEditMetricButton = new wxToggleButton( cellEditingSizer->GetStaticBox(), ID_LINE_EDIT_METRIC_BUTTON, wxT("Line"), wxDefaultPosition, wxDefaultSize, 0 );
	lineAndFloodButtonSizer->Add( lineEditMetricButton, 0, wxALL, 5 );

	floodEditMetricButton = new wxToggleButton( cellEditingSizer->GetStaticBox(), ID_FLOOD_EDIT_METRIC_BUTTON, wxT("Flood"), wxDefaultPosition, wxDefaultSize, 0 );
	lineAndFloodButtonSizer->Add( floodEditMetricButton, 0, wxALL, 5 );


	cellEditingSizer->Add( lineAndFloodButtonSizer, 1, wxEXPAND, 5 );

	wxBoxSizer* editingUndoSaveSizer;
	editingUndoSaveSizer = new wxBoxSizer( wxHORIZONTAL );

	undoCellsButton = new wxButton( cellEditingSizer->GetStaticBox(), ID_UNDO_CELLS_BUTTON, wxT("Undo"), wxDefaultPosition, wxDefaultSize, 0 );
	editingUndoSaveSizer->Add( undoCellsButton, 0, wxALL, 5 );

	saveCellsButton = new wxButton( cellEditingSizer->GetStaticBox(), ID_SAVE_CELLS_BUTTON, wxT("Save"), wxDefaultPosition, wxDefaultSize, 0 );
	editingUndoSaveSizer->Add( saveCellsButton, 0, wxALL, 5 );


	cellEditingSizer->Add( editingUndoSaveSizer, 0, 0, 5 );


	metricEditorOptionsSizer->Add( cellEditingSizer, 0, wxEXPAND, 5 );


	localMetricPanelSizer->Add( metricEditorOptionsSizer, 0, wxEXPAND, 5 );


	localMetricNotebookPanel->SetSizer( localMetricPanelSizer );
	localMetricNotebookPanel->Layout();
	localMetricPanelSizer->Fit( localMetricNotebookPanel );
	mapEditorNotebook->AddPage( localMetricNotebookPanel, wxT("Local Metric"), false, wxNullBitmap );
	localTopoNotebookPanel = new wxPanel( mapEditorNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* localTopoPanelSizer;
	localTopoPanelSizer = new wxBoxSizer( wxHORIZONTAL );

	wxBoxSizer* localTopoWidgetSizer;
	localTopoWidgetSizer = new wxBoxSizer( wxVERTICAL );

	localTopoEditorWidget = new LocalTopoEditorWidget(localTopoNotebookPanel, ID_LOCAL_TOPO_EDITOR_WIDGET, wxDefaultPosition,wxDefaultSize);
	localTopoWidgetSizer->Add( localTopoEditorWidget, 2, wxALL|wxEXPAND, 5 );

	wxBoxSizer* localTopoTrainingSizer;
	localTopoTrainingSizer = new wxBoxSizer( wxHORIZONTAL );

	wxStaticBoxSizer* labeledDataSizer;
	labeledDataSizer = new wxStaticBoxSizer( new wxStaticBox( localTopoNotebookPanel, wxID_ANY, wxT("Labeled Data:") ), wxVERTICAL );

	labelDataList = new wxListCtrl( labeledDataSizer->GetStaticBox(), ID_LABEL_DATA_LIST, wxDefaultPosition, wxDefaultSize, wxLC_HRULES|wxLC_REPORT|wxLC_VRULES );
	labeledDataSizer->Add( labelDataList, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* trainAndTestSizer;
	trainAndTestSizer = new wxBoxSizer( wxHORIZONTAL );


	trainAndTestSizer->Add( 0, 0, 1, wxEXPAND, 5 );

	loadLabelsButton = new wxButton( labeledDataSizer->GetStaticBox(), ID_LOAD_LABELS_BUTTON, wxT("Load Labels"), wxDefaultPosition, wxDefaultSize, 0 );
	trainAndTestSizer->Add( loadLabelsButton, 0, wxALL|wxEXPAND, 5 );

	saveLabelsButton = new wxButton( labeledDataSizer->GetStaticBox(), ID_SAVE_LABELS_BUTTON, wxT("Save Labels"), wxDefaultPosition, wxDefaultSize, 0 );
	trainAndTestSizer->Add( saveLabelsButton, 0, wxALL|wxEXPAND, 5 );

	trainGatewaysButton = new wxButton( labeledDataSizer->GetStaticBox(), ID_TRAIN_GATEWAYS_BUTTON, wxT("Train Gateways"), wxDefaultPosition, wxDefaultSize, 0 );
	trainAndTestSizer->Add( trainGatewaysButton, 0, wxALL, 5 );

	trainAndTestButton = new wxButton( labeledDataSizer->GetStaticBox(), ID_TRAIN_AND_TEST_BUTTON, wxT("Train Areas"), wxDefaultPosition, wxDefaultSize, 0 );
	trainAndTestSizer->Add( trainAndTestButton, 1, wxALL|wxEXPAND, 5 );


	trainAndTestSizer->Add( 0, 0, 1, wxEXPAND, 5 );


	labeledDataSizer->Add( trainAndTestSizer, 0, wxEXPAND, 5 );


	localTopoTrainingSizer->Add( labeledDataSizer, 2, wxEXPAND, 5 );

	wxStaticBoxSizer* trainingDataSizer;
	trainingDataSizer = new wxStaticBoxSizer( new wxStaticBox( localTopoNotebookPanel, wxID_ANY, wxT("Training Data:") ), wxVERTICAL );

	trainingDataList = new wxListBox( trainingDataSizer->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, wxLB_MULTIPLE );
	trainingDataSizer->Add( trainingDataList, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* trainingDataButtonsSizer;
	trainingDataButtonsSizer = new wxBoxSizer( wxHORIZONTAL );


	trainingDataButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );

	addTrainingDataButton = new wxButton( trainingDataSizer->GetStaticBox(), ID_ADD_TRAINING_DATA_BUTTON, wxT("  +  "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	trainingDataButtonsSizer->Add( addTrainingDataButton, 0, wxALL, 5 );

	removeTrainingDataButton = new wxButton( trainingDataSizer->GetStaticBox(), ID_REMOVE_TRAINING_DATA_BUTTON, wxT("  -  "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	trainingDataButtonsSizer->Add( removeTrainingDataButton, 0, wxALL, 5 );


	trainingDataButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );


	trainingDataSizer->Add( trainingDataButtonsSizer, 0, wxEXPAND, 5 );


	localTopoTrainingSizer->Add( trainingDataSizer, 1, wxEXPAND, 5 );

	wxStaticBoxSizer* testDataSizer;
	testDataSizer = new wxStaticBoxSizer( new wxStaticBox( localTopoNotebookPanel, wxID_ANY, wxT("Test Data:") ), wxVERTICAL );

	testDataList = new wxListBox( testDataSizer->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, wxLB_MULTIPLE );
	testDataSizer->Add( testDataList, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* testDataButtonsSizer;
	testDataButtonsSizer = new wxBoxSizer( wxHORIZONTAL );


	testDataButtonsSizer->Add( 0, 0, 1, 0, 5 );

	addTestDataButton = new wxButton( testDataSizer->GetStaticBox(), ID_ADD_TEST_DATA_BUTTON, wxT("  +  "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	testDataButtonsSizer->Add( addTestDataButton, 0, wxALL, 5 );

	removeTestDataButton = new wxButton( testDataSizer->GetStaticBox(), ID_REMOVE_TEST_DATA_BUTTON, wxT("  -  "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	testDataButtonsSizer->Add( removeTestDataButton, 0, wxALL, 5 );


	testDataButtonsSizer->Add( 0, 0, 1, 0, 5 );


	testDataSizer->Add( testDataButtonsSizer, 0, wxEXPAND, 5 );


	localTopoTrainingSizer->Add( testDataSizer, 1, wxEXPAND, 5 );


	localTopoWidgetSizer->Add( localTopoTrainingSizer, 1, wxEXPAND, 5 );


	localTopoPanelSizer->Add( localTopoWidgetSizer, 1, wxEXPAND, 5 );

	localTopoEditOptionsScroll = new wxScrolledWindow( localTopoNotebookPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL );
	localTopoEditOptionsScroll->SetScrollRate( 0, 5 );
	wxBoxSizer* localTopoEditorOptionsSizer;
	localTopoEditorOptionsSizer = new wxBoxSizer( wxVERTICAL );

	wxStaticBoxSizer* localTopoIOOptionsSIzer;
	localTopoIOOptionsSIzer = new wxStaticBoxSizer( new wxStaticBox( localTopoEditOptionsScroll, wxID_ANY, wxT("I/O Options") ), wxVERTICAL );

	loadLocalTopoLPMButton = new wxButton( localTopoIOOptionsSIzer->GetStaticBox(), ID_LOAD_LOCAL_TOPO_FROM_FILE_BUTTON, wxT("Load Map"), wxDefaultPosition, wxDefaultSize, 0 );
	localTopoIOOptionsSIzer->Add( loadLocalTopoLPMButton, 0, wxALL|wxEXPAND, 5 );

	storeLabelsButton = new wxButton( localTopoIOOptionsSIzer->GetStaticBox(), ID_STORE_LABELS_BUTTON, wxT("Store Map Info"), wxDefaultPosition, wxDefaultSize, 0 );
	storeLabelsButton->SetToolTip( wxT("Save the hand-created gateways and hand-labels areas associated with the current map.") );

	localTopoIOOptionsSIzer->Add( storeLabelsButton, 0, wxALL|wxEXPAND, 5 );

	currentMapNameLabel = new wxStaticText( localTopoIOOptionsSIzer->GetStaticBox(), wxID_ANY, wxT("Current Map:"), wxDefaultPosition, wxDefaultSize, 0 );
	currentMapNameLabel->Wrap( -1 );
	localTopoIOOptionsSIzer->Add( currentMapNameLabel, 0, wxALIGN_LEFT|wxALL, 5 );

	currentMapNameText = new wxStaticText( localTopoIOOptionsSIzer->GetStaticBox(), wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	currentMapNameText->Wrap( -1 );
	localTopoIOOptionsSIzer->Add( currentMapNameText, 0, wxALIGN_LEFT|wxALL, 5 );


	localTopoEditorOptionsSizer->Add( localTopoIOOptionsSIzer, 0, wxEXPAND, 5 );

	wxString localTopoEditModeRadioChoices[] = { wxT("Gateways"), wxT("Label"), wxT("Merge") };
	int localTopoEditModeRadioNChoices = sizeof( localTopoEditModeRadioChoices ) / sizeof( wxString );
	localTopoEditModeRadio = new wxRadioBox( localTopoEditOptionsScroll, ID_LOCAL_TOPO_EDIT_MODE_RADIO, wxT("Edit Mode"), wxDefaultPosition, wxDefaultSize, localTopoEditModeRadioNChoices, localTopoEditModeRadioChoices, 1, wxRA_SPECIFY_COLS );
	localTopoEditModeRadio->SetSelection( 0 );
	localTopoEditorOptionsSizer->Add( localTopoEditModeRadio, 0, wxALL|wxEXPAND, 5 );

	wxStaticBoxSizer* gatewayEditingOptionsSizer;
	gatewayEditingOptionsSizer = new wxStaticBoxSizer( new wxStaticBox( localTopoEditOptionsScroll, wxID_ANY, wxT("Gateway Options") ), wxVERTICAL );

	generateGatewaysButton = new wxButton( gatewayEditingOptionsSizer->GetStaticBox(), ID_AUTO_GENERATE_GATEWAYS_BUTTON, wxT("Auto-Generate"), wxDefaultPosition, wxDefaultSize, 0 );
	generateGatewaysButton->SetToolTip( wxT("Use the default gateway creation algorithm to create the gateways.") );

	gatewayEditingOptionsSizer->Add( generateGatewaysButton, 0, wxALL|wxEXPAND, 5 );

	handCreateGatewaysButton = new wxToggleButton( gatewayEditingOptionsSizer->GetStaticBox(), ID_HAND_CREATE_GATEWAYS_BUTTON, wxT("Hand-Create"), wxDefaultPosition, wxDefaultSize, 0 );
	gatewayEditingOptionsSizer->Add( handCreateGatewaysButton, 0, wxALL|wxEXPAND, 5 );

	loadGatewaysButton = new wxButton( gatewayEditingOptionsSizer->GetStaticBox(), ID_LOAD_GATEWAYS_BUTTON, wxT("Load From File"), wxDefaultPosition, wxDefaultSize, 0 );
	loadGatewaysButton->SetToolTip( wxT("Load the gateways from the .gwy file stored in the same directory as the LPM.") );

	gatewayEditingOptionsSizer->Add( loadGatewaysButton, 0, wxALL|wxEXPAND, 5 );

	createAreasFromGatewaysButton = new wxButton( gatewayEditingOptionsSizer->GetStaticBox(), ID_CREATE_AREAS_FROM_GATEWAYS_BUTTON, wxT("Create Areas"), wxDefaultPosition, wxDefaultSize, 0 );
	createAreasFromGatewaysButton->SetToolTip( wxT("Create areas from the gateways that have been created for the current map.") );

	gatewayEditingOptionsSizer->Add( createAreasFromGatewaysButton, 0, wxALL|wxEXPAND, 5 );


	localTopoEditorOptionsSizer->Add( gatewayEditingOptionsSizer, 0, wxEXPAND, 5 );

	wxStaticBoxSizer* localAreaLabelingOptionsSizer;
	localAreaLabelingOptionsSizer = new wxStaticBoxSizer( new wxStaticBox( localTopoEditOptionsScroll, wxID_ANY, wxT("Labeling Options") ), wxVERTICAL );

	wxString labelToAssignRadioChoices[] = { wxT("Path Segment"), wxT("Decision Point"), wxT("Destination") };
	int labelToAssignRadioNChoices = sizeof( labelToAssignRadioChoices ) / sizeof( wxString );
	labelToAssignRadio = new wxRadioBox( localAreaLabelingOptionsSizer->GetStaticBox(), ID_LABEL_TO_ASSIGN_RADIO, wxT("Label to Assign"), wxDefaultPosition, wxDefaultSize, labelToAssignRadioNChoices, labelToAssignRadioChoices, 1, wxRA_SPECIFY_COLS );
	labelToAssignRadio->SetSelection( 0 );
	localAreaLabelingOptionsSizer->Add( labelToAssignRadio, 0, wxALL|wxEXPAND, 5 );

	assignLabelsButton = new wxToggleButton( localAreaLabelingOptionsSizer->GetStaticBox(), ID_ASSIGN_LABELS_BUTTON, wxT("Assign Labels"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaLabelingOptionsSizer->Add( assignLabelsButton, 0, wxALL|wxEXPAND, 5 );

	labelAllAreasButton = new wxButton( localAreaLabelingOptionsSizer->GetStaticBox(), ID_LABEL_ALL_AREAS_BUTTON, wxT("Label Remaining"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaLabelingOptionsSizer->Add( labelAllAreasButton, 0, wxALL|wxEXPAND, 5 );

	clearLocalAreaLabelsButton = new wxButton( localAreaLabelingOptionsSizer->GetStaticBox(), ID_CLEAR_LOCAL_AREA_LABELS_BUTTON, wxT("Clear Labels"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaLabelingOptionsSizer->Add( clearLocalAreaLabelsButton, 0, wxALL|wxEXPAND, 5 );

	simplifyViaLabelsButton = new wxButton( localAreaLabelingOptionsSizer->GetStaticBox(), ID_SIMPLIFY_VIA_LABELS_BUTTON, wxT("Simplify Via Labels"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaLabelingOptionsSizer->Add( simplifyViaLabelsButton, 0, wxALL|wxEXPAND, 5 );


	localTopoEditorOptionsSizer->Add( localAreaLabelingOptionsSizer, 0, wxALL|wxEXPAND, 5 );

	wxStaticBoxSizer* localAreaMergeOptions;
	localAreaMergeOptions = new wxStaticBoxSizer( new wxStaticBox( localTopoEditOptionsScroll, wxID_ANY, wxT("Merge Options") ), wxVERTICAL );

	selectLocalAreasButton = new wxToggleButton( localAreaMergeOptions->GetStaticBox(), ID_SELECT_LOCAL_AREAS_BUTTON, wxT("Select Areas"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaMergeOptions->Add( selectLocalAreasButton, 0, wxALL|wxEXPAND, 5 );

	mergeSelectedAreasButton = new wxButton( localAreaMergeOptions->GetStaticBox(), ID_MERGE_SELECTED_AREAS_BUTTON, wxT("Merge Selected"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaMergeOptions->Add( mergeSelectedAreasButton, 0, wxALL|wxEXPAND, 5 );

	clearSelectedAreasButton = new wxButton( localAreaMergeOptions->GetStaticBox(), ID_CLEAR_SELECTED_AREAS_BUTTON, wxT("Clear Selected"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaMergeOptions->Add( clearSelectedAreasButton, 0, wxALL|wxEXPAND, 5 );

	resetMergedAreasButton = new wxButton( localAreaMergeOptions->GetStaticBox(), ID_RESET_MERGED_AREAS_BUTTON, wxT("Reset Merged"), wxDefaultPosition, wxDefaultSize, 0 );
	localAreaMergeOptions->Add( resetMergedAreasButton, 0, wxALL|wxEXPAND, 5 );


	localTopoEditorOptionsSizer->Add( localAreaMergeOptions, 0, wxALL|wxEXPAND, 5 );


	localTopoEditOptionsScroll->SetSizer( localTopoEditorOptionsSizer );
	localTopoEditOptionsScroll->Layout();
	localTopoEditorOptionsSizer->Fit( localTopoEditOptionsScroll );
	localTopoPanelSizer->Add( localTopoEditOptionsScroll, 0, wxALL|wxEXPAND, 5 );


	localTopoNotebookPanel->SetSizer( localTopoPanelSizer );
	localTopoNotebookPanel->Layout();
	localTopoPanelSizer->Fit( localTopoNotebookPanel );
	mapEditorNotebook->AddPage( localTopoNotebookPanel, wxT("Local Topo"), true, wxNullBitmap );
	globalTopoNotebookPanel = new wxPanel( mapEditorNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* globalTopoPanelSizer;
	globalTopoPanelSizer = new wxBoxSizer( wxHORIZONTAL );

	globalTopoEditorWidget = new GlobalTopoEditorWidget(globalTopoNotebookPanel, ID_GLOBAL_TOPO_EDITOR_WIDGET, wxDefaultPosition,wxDefaultSize);
	globalTopoPanelSizer->Add( globalTopoEditorWidget, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* globalTopoOptionsSizer;
	globalTopoOptionsSizer = new wxBoxSizer( wxVERTICAL );


	globalTopoPanelSizer->Add( globalTopoOptionsSizer, 0, wxEXPAND, 5 );


	globalTopoNotebookPanel->SetSizer( globalTopoPanelSizer );
	globalTopoNotebookPanel->Layout();
	globalTopoPanelSizer->Fit( globalTopoNotebookPanel );
	mapEditorNotebook->AddPage( globalTopoNotebookPanel, wxT("Global Topo"), false, wxNullBitmap );

	mapEditorSizer->Add( mapEditorNotebook, 1, wxEXPAND | wxALL, 5 );


	this->SetSizer( mapEditorSizer );
	this->Layout();
	editorStatusBar = this->CreateStatusBar( 1, wxSTB_SIZEGRIP, wxID_ANY );

	this->Centre( wxBOTH );
}

EditorFrame::~EditorFrame()
{
}

ImportImageDialogBase::ImportImageDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* importImageDialogSizer;
	importImageDialogSizer = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* selectImageSizer;
	selectImageSizer = new wxBoxSizer( wxHORIZONTAL );

	imageFilenameText = new wxTextCtrl( this, ID_IMAGE_FILENAME_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	selectImageSizer->Add( imageFilenameText, 1, wxALL, 5 );

	selectImageButton = new wxButton( this, ID_SELECT_IMAGE_BUTTON, wxT("Select Image"), wxDefaultPosition, wxDefaultSize, 0 );
	selectImageSizer->Add( selectImageButton, 0, wxALL, 5 );


	importImageDialogSizer->Add( selectImageSizer, 0, wxEXPAND, 5 );

	wxFlexGridSizer* imagePropertiesSizer;
	imagePropertiesSizer = new wxFlexGridSizer( 0, 2, 0, 0 );
	imagePropertiesSizer->SetFlexibleDirection( wxBOTH );
	imagePropertiesSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	imageDimensionsDescriptionLabel = new wxStaticText( this, wxID_ANY, wxT("Dimensions (cells):"), wxDefaultPosition, wxDefaultSize, 0 );
	imageDimensionsDescriptionLabel->Wrap( -1 );
	imagePropertiesSizer->Add( imageDimensionsDescriptionLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	imageDimensionsLabel = new wxStaticText( this, wxID_ANY, wxT("0x0"), wxDefaultPosition, wxDefaultSize, 0 );
	imageDimensionsLabel->Wrap( -1 );
	imagePropertiesSizer->Add( imageDimensionsLabel, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	cellScaleLabel = new wxStaticText( this, wxID_ANY, wxT("Scale (m/cell):"), wxDefaultPosition, wxDefaultSize, 0 );
	cellScaleLabel->Wrap( -1 );
	imagePropertiesSizer->Add( cellScaleLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	cellScaleText = new wxTextCtrl( this, ID_CELL_SCALE_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	imagePropertiesSizer->Add( cellScaleText, 1, wxALL|wxEXPAND, 5 );

	cellThresholdsLabel = new wxStaticText( this, wxID_ANY, wxT("Thresholds:"), wxDefaultPosition, wxDefaultSize, 0 );
	cellThresholdsLabel->Wrap( -1 );
	imagePropertiesSizer->Add( cellThresholdsLabel, 0, wxALIGN_RIGHT|wxALL, 5 );

	m_staticline1 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	imagePropertiesSizer->Add( m_staticline1, 1, wxEXPAND | wxALL, 5 );

	freeThresholdLabel = new wxStaticText( this, wxID_ANY, wxT("Free (0-255):"), wxDefaultPosition, wxDefaultSize, 0 );
	freeThresholdLabel->Wrap( -1 );
	imagePropertiesSizer->Add( freeThresholdLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	freeThresholdText = new wxTextCtrl( this, ID_FREE_THRESHOLD_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	imagePropertiesSizer->Add( freeThresholdText, 1, wxALL|wxEXPAND, 5 );

	occupiedThresholdLabel = new wxStaticText( this, wxID_ANY, wxT("Occupied (0-255):"), wxDefaultPosition, wxDefaultSize, 0 );
	occupiedThresholdLabel->Wrap( -1 );
	imagePropertiesSizer->Add( occupiedThresholdLabel, 0, wxALIGN_CENTER_VERTICAL|wxALIGN_RIGHT|wxALL, 5 );

	occupiedThresholdText = new wxTextCtrl( this, ID_OCCUPIED_THRESHOLD_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	imagePropertiesSizer->Add( occupiedThresholdText, 1, wxALL|wxEXPAND, 5 );

	importImageButton = new wxButton( this, ID_IMPORT_IMAGE_BUTTON, wxT("Import"), wxDefaultPosition, wxDefaultSize, 0 );
	imagePropertiesSizer->Add( importImageButton, 0, wxALIGN_RIGHT|wxALL, 5 );

	cancelImportButton = new wxButton( this, ID_CANCEL_IMPORT_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	imagePropertiesSizer->Add( cancelImportButton, 0, wxALIGN_LEFT|wxALL, 5 );


	importImageDialogSizer->Add( imagePropertiesSizer, 1, wxEXPAND, 5 );


	this->SetSizer( importImageDialogSizer );
	this->Layout();

	this->Centre( wxBOTH );
}

ImportImageDialogBase::~ImportImageDialogBase()
{
}

ClassificationTestResultsDialogBase::ClassificationTestResultsDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* classificationResultsSizer;
	classificationResultsSizer = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* summaryTitleSizer;
	summaryTitleSizer = new wxBoxSizer( wxHORIZONTAL );

	m_staticline4 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	summaryTitleSizer->Add( m_staticline4, 1, wxEXPAND | wxALL, 5 );

	classificationResultsSummaryLabel = new wxStaticText( this, wxID_ANY, wxT("Summary:"), wxDefaultPosition, wxDefaultSize, 0 );
	classificationResultsSummaryLabel->Wrap( -1 );
	classificationResultsSummaryLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	summaryTitleSizer->Add( classificationResultsSummaryLabel, 0, wxALL, 5 );

	m_staticline5 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	summaryTitleSizer->Add( m_staticline5, 1, wxEXPAND | wxALL, 5 );


	classificationResultsSizer->Add( summaryTitleSizer, 0, wxEXPAND, 5 );

	wxBoxSizer* trainingResultsSummarySizer;
	trainingResultsSummarySizer = new wxBoxSizer( wxHORIZONTAL );


	trainingResultsSummarySizer->Add( 0, 0, 1, wxEXPAND, 5 );

	trainingSetResultsLabel = new wxStaticText( this, wxID_ANY, wxT("Training Data:"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingSetResultsLabel->Wrap( -1 );
	trainingSetResultsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	trainingResultsSummarySizer->Add( trainingSetResultsLabel, 0, wxALL, 5 );

	trainingResultsTotalLabel = new wxStaticText( this, wxID_ANY, wxT("Total:"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsTotalLabel->Wrap( -1 );
	trainingResultsTotalLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	trainingResultsSummarySizer->Add( trainingResultsTotalLabel, 0, wxALL, 5 );

	trainingResultsTotalText = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsTotalText->Wrap( -1 );
	trainingResultsSummarySizer->Add( trainingResultsTotalText, 0, wxALL, 5 );

	trainingResultsCorrectLabel = new wxStaticText( this, wxID_ANY, wxT("Correct:"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsCorrectLabel->Wrap( -1 );
	trainingResultsCorrectLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	trainingResultsSummarySizer->Add( trainingResultsCorrectLabel, 0, wxALL, 5 );

	trainingResultsCorrectText = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsCorrectText->Wrap( -1 );
	trainingResultsSummarySizer->Add( trainingResultsCorrectText, 0, wxALL, 5 );

	trainingResultsAccuracyLabel = new wxStaticText( this, wxID_ANY, wxT("Accuracy:"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsAccuracyLabel->Wrap( -1 );
	trainingResultsAccuracyLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	trainingResultsSummarySizer->Add( trainingResultsAccuracyLabel, 0, wxALL, 5 );

	trainingResultsAccuracyText = new wxStaticText( this, wxID_ANY, wxT("0%"), wxDefaultPosition, wxDefaultSize, 0 );
	trainingResultsAccuracyText->Wrap( -1 );
	trainingResultsSummarySizer->Add( trainingResultsAccuracyText, 0, wxALL, 5 );


	trainingResultsSummarySizer->Add( 0, 0, 1, wxEXPAND, 5 );


	classificationResultsSizer->Add( trainingResultsSummarySizer, 0, wxEXPAND, 5 );

	wxBoxSizer* testResultsSummarySizer;
	testResultsSummarySizer = new wxBoxSizer( wxHORIZONTAL );


	testResultsSummarySizer->Add( 0, 0, 1, wxEXPAND, 5 );

	testDataResultsLabel = new wxStaticText( this, wxID_ANY, wxT("Test Data:"), wxDefaultPosition, wxDefaultSize, 0 );
	testDataResultsLabel->Wrap( -1 );
	testDataResultsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	testResultsSummarySizer->Add( testDataResultsLabel, 0, wxALL, 5 );

	totalClassificationTestsLabel = new wxStaticText( this, wxID_ANY, wxT("Total:"), wxDefaultPosition, wxDefaultSize, 0 );
	totalClassificationTestsLabel->Wrap( -1 );
	totalClassificationTestsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	testResultsSummarySizer->Add( totalClassificationTestsLabel, 0, wxALL, 5 );

	totalClassificationTestsText = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0 );
	totalClassificationTestsText->Wrap( -1 );
	testResultsSummarySizer->Add( totalClassificationTestsText, 0, wxALL, 5 );

	correctClassificationTestsLabel = new wxStaticText( this, wxID_ANY, wxT("Correct:"), wxDefaultPosition, wxDefaultSize, 0 );
	correctClassificationTestsLabel->Wrap( -1 );
	correctClassificationTestsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	testResultsSummarySizer->Add( correctClassificationTestsLabel, 0, wxALL, 5 );

	correctClassificationTestsText = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0 );
	correctClassificationTestsText->Wrap( -1 );
	testResultsSummarySizer->Add( correctClassificationTestsText, 0, wxALL, 5 );

	classificationAccuracyLabel = new wxStaticText( this, wxID_ANY, wxT("Accuracy:"), wxDefaultPosition, wxDefaultSize, 0 );
	classificationAccuracyLabel->Wrap( -1 );
	classificationAccuracyLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	testResultsSummarySizer->Add( classificationAccuracyLabel, 0, wxALL, 5 );

	classificationAccuracyText = new wxStaticText( this, wxID_ANY, wxT("0%"), wxDefaultPosition, wxDefaultSize, 0 );
	classificationAccuracyText->Wrap( -1 );
	testResultsSummarySizer->Add( classificationAccuracyText, 0, wxALL, 5 );


	testResultsSummarySizer->Add( 0, 0, 1, wxEXPAND, 5 );


	classificationResultsSizer->Add( testResultsSummarySizer, 0, wxEXPAND, 5 );

	wxBoxSizer* detailsSummarySizer;
	detailsSummarySizer = new wxBoxSizer( wxHORIZONTAL );

	m_staticline41 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	detailsSummarySizer->Add( m_staticline41, 1, wxEXPAND | wxALL, 5 );

	detailedResultsLabel = new wxStaticText( this, wxID_ANY, wxT("Details:"), wxDefaultPosition, wxDefaultSize, 0 );
	detailedResultsLabel->Wrap( -1 );
	detailedResultsLabel->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxEmptyString ) );

	detailsSummarySizer->Add( detailedResultsLabel, 0, wxALL, 5 );

	m_staticline51 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	detailsSummarySizer->Add( m_staticline51, 1, wxEXPAND | wxALL, 5 );


	classificationResultsSizer->Add( detailsSummarySizer, 0, wxEXPAND, 5 );

	classificationDetailsList = new wxDataViewListCtrl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxDV_HORIZ_RULES|wxDV_ROW_LINES );
	classificationResultsSizer->Add( classificationDetailsList, 1, wxALL|wxEXPAND, 5 );

	wxBoxSizer* classificationResultsButtonsSizer;
	classificationResultsButtonsSizer = new wxBoxSizer( wxHORIZONTAL );


	classificationResultsButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );

	saveClassifierButton = new wxButton( this, ID_SAVE_CLASSIFIER_BUTTON, wxT("Save Classifier"), wxDefaultPosition, wxDefaultSize, 0 );
	classificationResultsButtonsSizer->Add( saveClassifierButton, 0, wxALL, 5 );

	closeClassifierButton = new wxButton( this, ID_CLOSE_CLASSIFIER_BUTTON, wxT("Close"), wxDefaultPosition, wxDefaultSize, 0 );
	classificationResultsButtonsSizer->Add( closeClassifierButton, 0, wxALL, 5 );


	classificationResultsButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );


	classificationResultsSizer->Add( classificationResultsButtonsSizer, 0, wxEXPAND, 5 );


	this->SetSizer( classificationResultsSizer );
	this->Layout();

	this->Centre( wxBOTH );
}

ClassificationTestResultsDialogBase::~ClassificationTestResultsDialogBase()
{
}
