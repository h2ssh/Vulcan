/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_metric_panel.h
* \author   Collin Johnson
*
* Declaration of LocalMetricPanel.
*/

#ifndef UI_DEBUG_LOCAL_METRIC_PANEL_H
#define UI_DEBUG_LOCAL_METRIC_PANEL_H

#include "ui/common/metric_path_creator.h"
#include "ui/common/ui_panel.h"
#include "hssh/local_metric/lpm.h"
#include <wx/wx.h>
#include <memory>

namespace vulcan
{
namespace ui
{

class  LocalMetricPanelTextUpdater;
class  LocalMetricDisplayWidget;
class  GridCellSelector;
struct ui_params_t;

struct local_metric_panel_widgets_t
{
    LocalMetricDisplayWidget*    lpmWidget = nullptr;
    LocalMetricPanelTextUpdater* updater = nullptr;

    wxRadioBox* localizationModeCheck = nullptr;
    wxRadioBox* gridToShowRadio = nullptr;
    wxCheckBox* centerOnRobotCheck = nullptr;
    wxRadioBox* laserToShowRadio = nullptr;
    wxCheckBox* showRaysCheck = nullptr;
    wxCheckBox* showExtractedCheck = nullptr;
    wxCheckBox* showIntensityCheck = nullptr;
    wxCheckBox* showPoseTraceCheck = nullptr;
    wxCheckBox* showMotionTraceCheck = nullptr;
    wxCheckBox* showErrorCheck = nullptr;
    wxCheckBox* showParticlesCheck = nullptr;
    wxCheckBox* showGlassIntensityCheck = nullptr;
    wxCheckBox* showWallsCheck = nullptr;
    wxCheckBox* showAnglesCheck = nullptr;
    wxRadioBox* anglesToShowRadio = nullptr;
    wxSlider*   flattenThresholdSlider = nullptr;
    wxSlider*   highlyVisibleThresholdSlider = nullptr;
    wxTextCtrl* rotationAngleText = nullptr;
    wxFrame*    mainFrame = nullptr;
};

/**
* LocalMetricPanel handles all the events for the LPM display panel. It also produces metric_waypoint_path_t
* during the path creation mode.
*/
class LocalMetricPanel : public UIPanel
{
public:

    /**
    * Constructor for LocalMetricPanel.
    */
    LocalMetricPanel(const ui_params_t& params, const local_metric_panel_widgets_t& widgets);

    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

private:

    local_metric_panel_widgets_t widgets;
    system::ModuleCommunicator* consumer;
    
    // Event handlers
    void modeChanged(wxCommandEvent& event);
    void gridToShowChanged(wxCommandEvent& event);
    void laserToShowChanged(wxCommandEvent& event);
    void showLaserLinesChecked(wxCommandEvent& event);
    void showExtractedLinesChecked(wxCommandEvent& event);
    void showIntensityPlotsChecked(wxCommandEvent& event);
    void centerOnRobotChecked(wxCommandEvent& event);
    void showPoseTraceChecked(wxCommandEvent& event);
    void showMotionTraceChecked(wxCommandEvent& event);
    void showUncertaintyEllipseChecked(wxCommandEvent& event);
    void showParticlesChecked(wxCommandEvent& event);
    void clearPosesPressed(wxCommandEvent& event);
    void clearMotionPressed(wxCommandEvent& event);
    void showGlassIntensityChecked(wxCommandEvent& event);
    void showWallsChecked(wxCommandEvent& event);
    void showAnglesChecked(wxCommandEvent& event);
    void anglesToShowChanged(wxCommandEvent& event);
    void runFlattenMapPressed(wxCommandEvent& event);
    void runDynamicFilterPressed(wxCommandEvent& event);
    
    void rotateLPMPressed(wxCommandEvent& event);
    void saveLPMPressed(wxCommandEvent& event);
    void saveGlassPressed(wxCommandEvent& event);
    void loadGlassPressed(wxCommandEvent& event);
    void savePosesPressed(wxCommandEvent& event);
    void saveScansPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_LOCAL_METRIC_PANEL_H
