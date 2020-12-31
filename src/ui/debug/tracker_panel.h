/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tracker_panel.h
 * \author   Collin Johnson
 *
 * Declaration of TrackerPanel.
 */

#ifndef UI_DEBUG_TRACKER_PANEL_H
#define UI_DEBUG_TRACKER_PANEL_H

#include "ui/common/ui_panel.h"
#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class TrackerDisplayWidget;
struct ui_params_t;

struct tracker_panel_widgets_t
{
    TrackerDisplayWidget* displayWidget = nullptr;

    wxCheckBox* showLaserObjectsBox = nullptr;
    wxCheckBox* showLaserPointsBox = nullptr;
    wxCheckBox* showLaserUncertaintyBox = nullptr;
    wxRadioBox* boundaryToShowRadio = nullptr;

    wxCheckBox* showTrackedObjectsBox = nullptr;
    wxCheckBox* showAccelerationBox = nullptr;
    wxRadioBox* uncertaintyToShowRadio = nullptr;
    wxCheckBox* showRecentTrajBox = nullptr;

    wxRadioBox* goalToShowRadio = nullptr;

    wxRadioBox* predictionToShowRadio = nullptr;
    wxTextCtrl* predictedTrajDurationText = nullptr;
};

/**
 * TrackerPanel
 */
class TrackerPanel : public UIPanel
{
public:
    /**
     * Constructor for TrackerPanel.
     *
     * \param    params          Parameters controlling the rendering
     * \param    widgets         Widgets in the panel
     */
    TrackerPanel(const ui_params_t& params, const tracker_panel_widgets_t& widgets);

    // UIPanel interface
    virtual void setup(wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe(system::ModuleCommunicator& producer);
    virtual void setConsumer(system::ModuleCommunicator* consumer);
    virtual void update(void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

private:
    tracker_panel_widgets_t widgets_;

    void followRobotChecked(wxCommandEvent& event);

    void showLaserObjectsChecked(wxCommandEvent& event);
    void showLaserPointsChecked(wxCommandEvent& event);
    void showLaserUncertaintyChecked(wxCommandEvent& event);
    void boundaryToShowChanged(wxCommandEvent& event);

    void showTrackedObjectsChecked(wxCommandEvent& event);
    void showAccelerationChecked(wxCommandEvent& event);
    void rigidObjectStateToShowChanged(wxCommandEvent& event);
    void trackingUncertaintyToShowChanged(wxCommandEvent& event);
    void showRecentTrajectoryChecked(wxCommandEvent& event);

    void goalToShowChanged(wxCommandEvent& event);
    void evaluateGoalsPressed(wxCommandEvent& event);

    void predictedTrajToShowChanged(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_TRACKER_PANEL_H
