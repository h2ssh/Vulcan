/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_editor_panel.h
 * \author   Collin Johnson
 *
 * Definition of MetricEditorPanel.
 */

#ifndef UI_MAPEDITOR_METRIC_EDITOR_PANEL_H
#define UI_MAPEDITOR_METRIC_EDITOR_PANEL_H

#include "ui/common/ui_panel.h"
#include <atomic>
#include <wx/radiobox.h>
#include <wx/tglbtn.h>

namespace vulcan
{
namespace hssh
{
class LocalPerceptualMap;
}

namespace ui
{

class GridCellSelector;
class LPMEditor;
class MetricEditorWidget;
struct ui_params_t;

struct metric_editor_panel_widgets_t
{
    MetricEditorWidget* editorWidget;
    wxRadioBox* editCellTypeRadio;
    wxToggleButton* cellEditModeButton;
    wxToggleButton* lineEditModeButton;
    wxToggleButton* floodEditModeButton;
};

/**
 * MetricEditorPanel
 */
class MetricEditorPanel : public UIPanel
{
public:
    /**
     * Constructor for MetricEditorPanel.
     *
     * \param    params          Parameters governing behavior of the panel
     * \param    widgets         Stateful widgets contained in the panel
     */
    MetricEditorPanel(const ui_params_t& params, const metric_editor_panel_widgets_t& widgets);

    /**
     * Destructor for MetricEditorPanel.
     */
    virtual ~MetricEditorPanel(void);

    // UIPanel interface
    virtual void setup(wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe(system::ModuleCommunicator& producer);
    virtual void setConsumer(system::ModuleCommunicator* consumer);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);
    virtual void update(void);

    // Data handlers
    void handleData(const hssh::LocalPerceptualMap& map, const std::string& channel);

private:
    enum class EditMode
    {
        NONE,   // currently off
        CELL,
        LINE,
        FILL
    };

    metric_editor_panel_widgets_t widgets;

    std::shared_ptr<hssh::LocalPerceptualMap> lpm;
    std::unique_ptr<LPMEditor> editor;

    std::shared_ptr<GridCellSelector> cellSelector;
    system::ModuleCommunicator* consumer;
    std::atomic<bool> shouldCaptureNextLPM;

    EditMode mode;
    int cellEditType;

    void changeDisplayedLPM(const hssh::LocalPerceptualMap& newLPM);

    // Event handlers
    void loadFromFilePressed(wxCommandEvent& event);
    void captureFromLCMPressed(wxCommandEvent& event);
    void createEmptyLPMPressed(wxCommandEvent& event);
    void saveToFilePressed(wxCommandEvent& event);
    void sendViaLCMPressed(wxCommandEvent& event);
    void importFromImagePressed(wxCommandEvent& event);
    void cellTypeChanged(wxCommandEvent& event);
    void editCellsPressed(wxCommandEvent& event);
    void lineModePressed(wxCommandEvent& event);
    void fillModePressed(wxCommandEvent& event);
    void undoEditPressed(wxCommandEvent& event);
    void commitEditsPressed(wxCommandEvent& event);

    void updateEditMode(void);
    void setSelectionInfo(void);
    void createEditsIfNecessary(void);
    void setCellHover(void);
    void setLineHover(void);
    void createCellEdit(void);
    void createLineEdit(void);
    void createFillEdit(void);
    void enableEditing(void);
    void disableEditing(void);

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_MAPEDITOR_METRIC_EDITOR_PANEL_H
