///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
namespace vulcan
{
namespace ui
{
class GlobalTopoEditorWidget;
}
}   // namespace vulcan
namespace vulcan
{
namespace ui
{
class LocalTopoEditorWidget;
}
}   // namespace vulcan
namespace vulcan
{
namespace ui
{
class MetricEditorWidget;
}
}   // namespace vulcan

#include "ui/common/ui_main_frame.h"
#include <wx/aui/auibook.h>
#include <wx/bitmap.h>
#include <wx/button.h>
#include <wx/colour.h>
#include <wx/dataview.h>
#include <wx/dialog.h>
#include <wx/font.h>
#include <wx/frame.h>
#include <wx/gdicmn.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/listbox.h>
#include <wx/listctrl.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/scrolwin.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/statusbr.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/tglbtn.h>

///////////////////////////////////////////////////////////////////////////

namespace vulcan
{
namespace ui
{
#define ID_METRIC_EDITOR_WIDGET 1000
#define ID_LOAD_METRIC_FROM_FILE_BUTTON 1001
#define ID_CAPTURE_METRIC_FROM_LCM_BUTTON 1002
#define ID_CREATE_METRIC_BUTTON 1003
#define ID_SAVE_TO_FILE_METRIC_BUTTON 1004
#define ID_SEND_VIA_LCM_METRIC_BUTTON 1005
#define ID_IMPORT_FROM_IMAGE_BUTTON 1006
#define ID_EDIT_CELL_TYPE_RADIO_BOX 1007
#define ID_CELL_EDIT_MODE_BUTTON 1008
#define ID_LINE_EDIT_METRIC_BUTTON 1009
#define ID_FLOOD_EDIT_METRIC_BUTTON 1010
#define ID_UNDO_CELLS_BUTTON 1011
#define ID_SAVE_CELLS_BUTTON 1012
#define ID_LOCAL_TOPO_EDITOR_WIDGET 1013
#define ID_LABEL_DATA_LIST 1014
#define ID_LOAD_LABELS_BUTTON 1015
#define ID_SAVE_LABELS_BUTTON 1016
#define ID_TRAIN_GATEWAYS_BUTTON 1017
#define ID_TRAIN_AND_TEST_BUTTON 1018
#define ID_ADD_TRAINING_DATA_BUTTON 1019
#define ID_REMOVE_TRAINING_DATA_BUTTON 1020
#define ID_ADD_TEST_DATA_BUTTON 1021
#define ID_REMOVE_TEST_DATA_BUTTON 1022
#define ID_LOAD_LOCAL_TOPO_FROM_FILE_BUTTON 1023
#define ID_STORE_LABELS_BUTTON 1024
#define ID_LOCAL_TOPO_EDIT_MODE_RADIO 1025
#define ID_AUTO_GENERATE_GATEWAYS_BUTTON 1026
#define ID_HAND_CREATE_GATEWAYS_BUTTON 1027
#define ID_LOAD_GATEWAYS_BUTTON 1028
#define ID_CREATE_AREAS_FROM_GATEWAYS_BUTTON 1029
#define ID_LABEL_TO_ASSIGN_RADIO 1030
#define ID_ASSIGN_LABELS_BUTTON 1031
#define ID_LABEL_ALL_AREAS_BUTTON 1032
#define ID_CLEAR_LOCAL_AREA_LABELS_BUTTON 1033
#define ID_SIMPLIFY_VIA_LABELS_BUTTON 1034
#define ID_SELECT_LOCAL_AREAS_BUTTON 1035
#define ID_MERGE_SELECTED_AREAS_BUTTON 1036
#define ID_CLEAR_SELECTED_AREAS_BUTTON 1037
#define ID_RESET_MERGED_AREAS_BUTTON 1038
#define ID_GLOBAL_TOPO_EDITOR_WIDGET 1039
#define ID_IMAGE_FILENAME_TEXT 1040
#define ID_SELECT_IMAGE_BUTTON 1041
#define ID_CELL_SCALE_TEXT 1042
#define ID_FREE_THRESHOLD_TEXT 1043
#define ID_OCCUPIED_THRESHOLD_TEXT 1044
#define ID_IMPORT_IMAGE_BUTTON 1045
#define ID_CANCEL_IMPORT_BUTTON 1046
#define ID_SAVE_CLASSIFIER_BUTTON 1047
#define ID_CLOSE_CLASSIFIER_BUTTON 1048

///////////////////////////////////////////////////////////////////////////////
/// Class EditorFrame
///////////////////////////////////////////////////////////////////////////////
class EditorFrame : public UIMainFrame
{
private:
protected:
    wxAuiNotebook* mapEditorNotebook;
    wxPanel* localMetricNotebookPanel;
    MetricEditorWidget* metricEditorWidget;
    wxButton* loadMetricFromFileButton;
    wxButton* captureMetricFromLCMButton;
    wxButton* createMetricButton;
    wxButton* saveToFileMetricButton;
    wxButton* sendViaLCMMetricButton;
    wxButton* importFromImageButton;
    wxRadioBox* editCellTypeRadioBox;
    wxToggleButton* cellEditModeButton;
    wxToggleButton* lineEditMetricButton;
    wxToggleButton* floodEditMetricButton;
    wxButton* undoCellsButton;
    wxButton* saveCellsButton;
    wxPanel* localTopoNotebookPanel;
    LocalTopoEditorWidget* localTopoEditorWidget;
    wxListCtrl* labelDataList;
    wxButton* loadLabelsButton;
    wxButton* saveLabelsButton;
    wxButton* trainGatewaysButton;
    wxButton* trainAndTestButton;
    wxListBox* trainingDataList;
    wxButton* addTrainingDataButton;
    wxButton* removeTrainingDataButton;
    wxListBox* testDataList;
    wxButton* addTestDataButton;
    wxButton* removeTestDataButton;
    wxScrolledWindow* localTopoEditOptionsScroll;
    wxButton* loadLocalTopoLPMButton;
    wxButton* storeLabelsButton;
    wxStaticText* currentMapNameLabel;
    wxStaticText* currentMapNameText;
    wxRadioBox* localTopoEditModeRadio;
    wxButton* generateGatewaysButton;
    wxToggleButton* handCreateGatewaysButton;
    wxButton* loadGatewaysButton;
    wxButton* createAreasFromGatewaysButton;
    wxRadioBox* labelToAssignRadio;
    wxToggleButton* assignLabelsButton;
    wxButton* labelAllAreasButton;
    wxButton* clearLocalAreaLabelsButton;
    wxButton* simplifyViaLabelsButton;
    wxToggleButton* selectLocalAreasButton;
    wxButton* mergeSelectedAreasButton;
    wxButton* clearSelectedAreasButton;
    wxButton* resetMergedAreasButton;
    wxPanel* globalTopoNotebookPanel;
    GlobalTopoEditorWidget* globalTopoEditorWidget;
    wxStatusBar* editorStatusBar;

public:
    EditorFrame(wxWindow* parent,
                wxWindowID id = wxID_ANY,
                const wxString& title = wxT("Map Editor"),
                const wxPoint& pos = wxDefaultPosition,
                const wxSize& size = wxSize(1094, 862),
                long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);

    ~EditorFrame();
};

///////////////////////////////////////////////////////////////////////////////
/// Class ImportImageDialogBase
///////////////////////////////////////////////////////////////////////////////
class ImportImageDialogBase : public wxDialog
{
private:
protected:
    wxTextCtrl* imageFilenameText;
    wxButton* selectImageButton;
    wxStaticText* imageDimensionsDescriptionLabel;
    wxStaticText* imageDimensionsLabel;
    wxStaticText* cellScaleLabel;
    wxTextCtrl* cellScaleText;
    wxStaticText* cellThresholdsLabel;
    wxStaticLine* m_staticline1;
    wxStaticText* freeThresholdLabel;
    wxTextCtrl* freeThresholdText;
    wxStaticText* occupiedThresholdLabel;
    wxTextCtrl* occupiedThresholdText;
    wxButton* importImageButton;
    wxButton* cancelImportButton;

public:
    ImportImageDialogBase(wxWindow* parent,
                          wxWindowID id = wxID_ANY,
                          const wxString& title = wxT("Import LPM From Image..."),
                          const wxPoint& pos = wxDefaultPosition,
                          const wxSize& size = wxSize(250, 282),
                          long style = wxDEFAULT_DIALOG_STYLE);
    ~ImportImageDialogBase();
};

///////////////////////////////////////////////////////////////////////////////
/// Class ClassificationTestResultsDialogBase
///////////////////////////////////////////////////////////////////////////////
class ClassificationTestResultsDialogBase : public wxDialog
{
private:
protected:
    wxStaticLine* m_staticline4;
    wxStaticText* classificationResultsSummaryLabel;
    wxStaticLine* m_staticline5;
    wxStaticText* trainingSetResultsLabel;
    wxStaticText* trainingResultsTotalLabel;
    wxStaticText* trainingResultsTotalText;
    wxStaticText* trainingResultsCorrectLabel;
    wxStaticText* trainingResultsCorrectText;
    wxStaticText* trainingResultsAccuracyLabel;
    wxStaticText* trainingResultsAccuracyText;
    wxStaticText* testDataResultsLabel;
    wxStaticText* totalClassificationTestsLabel;
    wxStaticText* totalClassificationTestsText;
    wxStaticText* correctClassificationTestsLabel;
    wxStaticText* correctClassificationTestsText;
    wxStaticText* classificationAccuracyLabel;
    wxStaticText* classificationAccuracyText;
    wxStaticLine* m_staticline41;
    wxStaticText* detailedResultsLabel;
    wxStaticLine* m_staticline51;
    wxDataViewListCtrl* classificationDetailsList;
    wxButton* saveClassifierButton;
    wxButton* closeClassifierButton;

public:
    ClassificationTestResultsDialogBase(wxWindow* parent,
                                        wxWindowID id = wxID_ANY,
                                        const wxString& title = wxT("Classification Results"),
                                        const wxPoint& pos = wxDefaultPosition,
                                        const wxSize& size = wxSize(1179, 428),
                                        long style = wxDEFAULT_DIALOG_STYLE);
    ~ClassificationTestResultsDialogBase();
};

}   // namespace ui
}   // namespace vulcan
