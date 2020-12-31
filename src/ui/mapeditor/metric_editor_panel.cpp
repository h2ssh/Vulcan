/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_editor_panel.cpp
 * \author   Collin Johnson
 *
 * Implementation of MetricEditorPanel.
 */

#include "ui/mapeditor/metric_editor_panel.h"
#include "hssh/local_metric/commands/set_map.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "hssh/local_metric/pose.h"
#include "system/module_communicator.h"
#include "ui/common/file_dialog_settings.h"
#include "ui/common/grid_cell_selector.h"
#include "ui/mapeditor/import_image_dialog.h"
#include "ui/mapeditor/lpm_editing.h"
#include "ui/mapeditor/map_editor.h"
#include "ui/mapeditor/metric_editor_widget.h"
#include <wx/filedlg.h>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(MetricEditorPanel, wxEvtHandler)
EVT_BUTTON(ID_LOAD_METRIC_FROM_FILE_BUTTON, MetricEditorPanel::loadFromFilePressed)
EVT_BUTTON(ID_CAPTURE_METRIC_FROM_LCM_BUTTON, MetricEditorPanel::captureFromLCMPressed)
EVT_BUTTON(ID_CREATE_METRIC_BUTTON, MetricEditorPanel::createEmptyLPMPressed)
EVT_BUTTON(ID_SAVE_TO_FILE_METRIC_BUTTON, MetricEditorPanel::saveToFilePressed)
EVT_BUTTON(ID_SEND_VIA_LCM_METRIC_BUTTON, MetricEditorPanel::sendViaLCMPressed)
EVT_BUTTON(ID_IMPORT_FROM_IMAGE_BUTTON, MetricEditorPanel::importFromImagePressed)
EVT_RADIOBOX(ID_EDIT_CELL_TYPE_RADIO_BOX, MetricEditorPanel::cellTypeChanged)
EVT_TOGGLEBUTTON(ID_CELL_EDIT_MODE_BUTTON, MetricEditorPanel::editCellsPressed)
EVT_TOGGLEBUTTON(ID_LINE_EDIT_METRIC_BUTTON, MetricEditorPanel::lineModePressed)
EVT_TOGGLEBUTTON(ID_FLOOD_EDIT_METRIC_BUTTON, MetricEditorPanel::fillModePressed)
EVT_BUTTON(ID_UNDO_CELLS_BUTTON, MetricEditorPanel::undoEditPressed)
EVT_BUTTON(ID_SAVE_CELLS_BUTTON, MetricEditorPanel::commitEditsPressed)
END_EVENT_TABLE()

// For Kdevelop code completion, which doesn't appreciate the EVT_TOGGLEBUTTON
using namespace ui;


hssh::cell_type_t radio_selection_to_cell_type(int selection);


MetricEditorPanel::MetricEditorPanel(const ui_params_t& params, const metric_editor_panel_widgets_t& widgets)
: widgets(widgets)
, editor(new LPMEditor)
, cellSelector(new GridCellSelector(widgets.editorWidget))
, shouldCaptureNextLPM(false)
, mode(EditMode::NONE)
{
    widgets.floodEditModeButton->Disable();
    widgets.lineEditModeButton->Disable();

    widgets.editorWidget->setParams(params);
}


MetricEditorPanel::~MetricEditorPanel(void)
{
}


void MetricEditorPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets.editorWidget->setStatusBar(statusBar);
    widgets.editorWidget->setRenderContext(context);
}


void MetricEditorPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalPerceptualMap>(this);
}


void MetricEditorPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    this->consumer = consumer;
}


void MetricEditorPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void MetricEditorPanel::loadSettings(const utils::ConfigFile& config)
{
}


void MetricEditorPanel::update(void)
{
    if (mode != EditMode::NONE) {
        updateEditMode();
    }

    widgets.editorWidget->Refresh();
}


void MetricEditorPanel::handleData(const hssh::LocalPerceptualMap& map, const std::string& channel)
{
    if (shouldCaptureNextLPM) {
        changeDisplayedLPM(map);
        shouldCaptureNextLPM = false;
    }
}


void MetricEditorPanel::changeDisplayedLPM(const hssh::LocalPerceptualMap& newLPM)
{
    lpm.reset(new hssh::LocalPerceptualMap(newLPM));
    widgets.editorWidget->setLPM(lpm);
    editor->setLPM(lpm.get());
}


void MetricEditorPanel::loadFromFilePressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.editorWidget,
                            wxT("Select map file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.lpm"),
                            kFileOpenFlags);

    if (loadDialog.ShowModal() == wxID_OK) {
        wxString path = loadDialog.GetPath();

        hssh::LocalPerceptualMap newLPM;
        hssh::load_lpm_1_0(std::string(path.mb_str()), newLPM);
        changeDisplayedLPM(newLPM);
    }
}


void MetricEditorPanel::captureFromLCMPressed(wxCommandEvent& event)
{
    shouldCaptureNextLPM = true;
}


void MetricEditorPanel::createEmptyLPMPressed(wxCommandEvent& event)
{
}


void MetricEditorPanel::saveToFilePressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets.editorWidget,
                            wxT("Select map file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.lpm"),
                            kFileSaveFlags);

    if (saveDialog.ShowModal() == wxID_OK) {
        wxString path = saveDialog.GetPath();
        hssh::save_lpm_1_0(*lpm, std::string(path.mb_str()));
    }
}


void MetricEditorPanel::sendViaLCMPressed(wxCommandEvent& event)
{
    if (lpm) {
        //         pose_t defaultPose(lpm->getGlobalCenter());
        //         hssh::LocalPose localDefaultPose(pose_distribution_t(defaultPose), lpm->getReferenceFrameIndex());
        //         defaultPose.referenceIndex = lpm->getReferenceFrameIndex();
        consumer->sendMessage(*lpm);
        //         consumer->sendMessage(defaultPose);
        //         consumer->sendMessage(localDefaultPose);

        // Also send a setMap message to update the map being used by local_metric_hssh to allow for fast annotations
        std::shared_ptr<hssh::LocalMetricCommand> command = std::make_shared<hssh::SetMapCommand>(*lpm, "MapEditor");
        consumer->sendMessage(command);
    }
}


void MetricEditorPanel::importFromImagePressed(wxCommandEvent& event)
{
    ImportImageDialog dialog(widgets.editorWidget);

    if (dialog.ShowModal() == wxID_OK) {
        changeDisplayedLPM(dialog.getImportedLPM());
    }
}


void MetricEditorPanel::cellTypeChanged(wxCommandEvent& event)
{
    cellEditType = widgets.editCellTypeRadio->GetSelection();
}


void MetricEditorPanel::editCellsPressed(wxCommandEvent& event)
{
    if (event.IsChecked()) {
        enableEditing();
    } else {
        disableEditing();
    }
}


void MetricEditorPanel::lineModePressed(wxCommandEvent& event)
{
    // Must be in edit mode for these buttons to even be enabled
    if (event.IsChecked()) {
        assert(widgets.cellEditModeButton->IsEnabled());

        widgets.floodEditModeButton->SetValue(false);   // turn off flood mode if going into line mode
        mode = EditMode::LINE;
    } else   // if unchecking, then go back to cell editing
    {
        mode = EditMode::CELL;
    }
}


void MetricEditorPanel::fillModePressed(wxCommandEvent& event)
{
    // Must be in edit mode for these buttons to even be enabled
    if (event.IsChecked()) {
        assert(widgets.cellEditModeButton->IsEnabled());

        widgets.lineEditModeButton->SetValue(false);   // turn off line editing if going into fill mode
        mode = EditMode::FILL;
    } else   // if unchecking, then go back to cell editing
    {
        mode = EditMode::CELL;
    }
}


void MetricEditorPanel::undoEditPressed(wxCommandEvent& event)
{
    editor->undo();
    widgets.editorWidget->changedLPM();
}


void MetricEditorPanel::commitEditsPressed(wxCommandEvent& event)
{
    editor->commit();
}


void MetricEditorPanel::updateEditMode(void)
{
    setSelectionInfo();
    createEditsIfNecessary();
}


void MetricEditorPanel::setSelectionInfo(void)
{
    switch (mode) {
    case EditMode::CELL:
        setCellHover();
        widgets.editorWidget->setSelectedCells(cellSelector->selectedCells());
        break;

    case EditMode::FILL:
        setCellHover();
        break;

    case EditMode::LINE:
        setLineHover();
        break;

    case EditMode::NONE:
    default:
        // There is no hover cell
        break;
    }
}


void MetricEditorPanel::createEditsIfNecessary(void)
{
    // Only create a new edit if there are selected cells AND the selection isn't currently being updated
    if (cellSelector->isActivelySelecting() || (cellSelector->numSelectedCells() == 0)) {
        return;
    }

    switch (mode) {
    case EditMode::CELL:
        createCellEdit();
        break;

    case EditMode::LINE:
        createLineEdit();
        break;

    case EditMode::FILL:
        createFillEdit();
        break;

    case EditMode::NONE:
    default:
        assert("ERROR::MetricEditorPanel: Trying to create edits when not in an editing mode.\n" && false);
    }

    // These cells have been incorporated into an edit, so they can be flushed out now
    cellSelector->clearSelectedCells();
}


void MetricEditorPanel::setCellHover(void)
{
    // For fill and cell modes, always just use the hover cell
    widgets.editorWidget->setHoverCell(cellSelector->hoverCell(), radio_selection_to_cell_type(cellEditType));
}


void MetricEditorPanel::setLineHover(void)
{
    // If not selecting, then there is no hover line to be drawn, so just draw the current cell
    if (!cellSelector->isActivelySelecting() || (cellSelector->numSelectedCells() == 0)) {
        setCellHover();
    } else   // The first and last selected cells define the line
    {
        auto selected = cellSelector->selectedCells();
        assert(!selected.empty());

        widgets.editorWidget->setHoverLine(Line<int>(selected.front(), selected.back()),
                                           radio_selection_to_cell_type(cellEditType));
    }
}


void MetricEditorPanel::createCellEdit(void)
{
    // For cell edits, take all the selected cells and create a new group for the edit
    editor->changeGroup(cellSelector->selectedCells(), radio_selection_to_cell_type(cellEditType));
    widgets.editorWidget->changedLPM();
}


void MetricEditorPanel::createLineEdit(void)
{
    // For a line edit, the line to create is defined by the first and last cells in the selection
    auto selected = cellSelector->selectedCells();

    editor->changeLine(Line<int>(selected.front(), selected.back()), radio_selection_to_cell_type(cellEditType));
    widgets.editorWidget->changedLPM();
}


void MetricEditorPanel::createFillEdit(void)
{
    // For a fill edit, take the last cell that was selected. The cell when the mouse up happened.
    auto selected = cellSelector->selectedCells();

    editor->changeRegion(selected.back(), radio_selection_to_cell_type(cellEditType));
    widgets.editorWidget->changedLPM();
}


void MetricEditorPanel::enableEditing(void)
{
    mode = EditMode::CELL;
    cellEditType = widgets.editCellTypeRadio->GetSelection();

    widgets.floodEditModeButton->Enable();
    widgets.lineEditModeButton->Enable();

    cellSelector->clearSelectedCells();
    widgets.editorWidget->setSelectedCells(cellSelector->selectedCells());
    widgets.editorWidget->shouldRenderHover(true);
    widgets.editorWidget->pushMouseHandler(cellSelector.get());
}


void MetricEditorPanel::disableEditing(void)
{
    mode = EditMode::NONE;

    widgets.floodEditModeButton->Disable();
    widgets.lineEditModeButton->Disable();

    widgets.floodEditModeButton->SetValue(false);
    widgets.lineEditModeButton->SetValue(false);

    widgets.editorWidget->shouldRenderHover(false);
    widgets.editorWidget->removeMouseHandler(cellSelector.get());
}


hssh::cell_type_t radio_selection_to_cell_type(int selection)
{
    switch (selection) {
    case 0:
        return hssh::kHazardOccGridCell;

    case 1:
        return hssh::kQuasiStaticOccGridCell;

    case 2:
        return hssh::kOccupiedOccGridCell;

    case 3:
        return hssh::kFreeOccGridCell;

    case 4:
        return hssh::kUnobservedOccGridCell;

    default:
        return hssh::kOccupiedOccGridCell;
    }
}

}   // namespace ui
}   // namespace vulcan
