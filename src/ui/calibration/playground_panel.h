/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     playground_panel.h
 * \author   Collin Johnson
 *
 * Declaration of PlaygroundPanel.
 */

#ifndef UI_CALIBRATION_PLAYGROUND_PANEL_H
#define UI_CALIBRATION_PLAYGROUND_PANEL_H

#include "ui/common/lpm_data_consumer.h"
#include "ui/common/ui_panel.h"

namespace vulcan
{
namespace ui
{

class PlaygroundDisplayWidget;
struct calibration_ui_params_t;

/**
 * playground_panel_widgets_t defines all the widgets controlled by the PlaygroundPanel.
 * These are the widgets from which the panel needs to read information, or to which the
 * panel will write information.
 */
struct playground_panel_widgets_t
{
    wxTextCtrl* vMaxText;
    wxTextCtrl* numTargetsText;
    wxTextCtrl* timePerTargetText;
    wxTextCtrl* logNameText;
    PlaygroundDisplayWidget* displayWidget;

    playground_panel_widgets_t(void)
    : vMaxText(0)
    , numTargetsText(0)
    , timePerTargetText(0)
    , logNameText(0)
    , displayWidget(0)
    {
    }
};

/**
 * control_law_dialog_widgets_t contains pointers to all widgets used for setting the parameters
 * in the control law dialog.
 */
struct control_law_dialog_widgets_t
{
    wxTextCtrl* k1Text;
    wxTextCtrl* k2Text;
    wxTextCtrl* maxLinearVelText;
    wxTextCtrl* maxAngularVelText;
    wxTextCtrl* angVelAtTargetText;
    wxTextCtrl* slowdownRadiusText;
    wxTextCtrl* convergenceRadiusText;
    wxTextCtrl* convergenceAngleText;
    wxTextCtrl* betaText;
    wxTextCtrl* lambdaText;

    control_law_dialog_widgets_t(void)
    : k1Text(0)
    , k2Text(0)
    , maxLinearVelText(0)
    , maxAngularVelText(0)
    , angVelAtTargetText(0)
    , slowdownRadiusText(0)
    , convergenceRadiusText(0)
    , convergenceAngleText(0)
    , betaText(0)
    , lambdaText(0)
    {
    }
};


/**
 * PlaygroundPanel handles the conversion of UI events into PlaygroundTargetGenerator inputs and
 * PlaygroundTargetGenerator outputs into UI displays.
 *
 * The panel also handles events for all the buttons on the Playground tab of the Calibration UI.
 */
class PlaygroundPanel
: public UIPanel
, public LPMDataConsumer
{
public:
    /**
     * Constructor for PlaygroundPanel.
     *
     * \param    widgets             Widgets to be manipulated by the panel
     * \param    params              Parameters for the panel and the playground
     */
    PlaygroundPanel(const playground_panel_widgets_t& widgets,
                    const control_law_dialog_widgets_t& controlWidgets,
                    const calibration_ui_params_t& params);

    // UIPanel interface
    virtual void setup(wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe(system::ModuleCommunicator& producer);
    virtual void setConsumer(system::ModuleCommunicator* consumer);
    virtual void update(void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

    // LPMDataConsumer interface
    virtual void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    virtual void handleData(const pose_t& pose, const std::string& channel);
    virtual void handleData(const velocity_t& velocity, const std::string& channel);
    virtual void handleData(const mpepc::metric_waypoint_path_t& path, const std::string& channel);

private:
    // Event handlers
    void createControllerPressed(wxCommandEvent& event);
    void loadControllerPressed(wxCommandEvent& event);
    void saveControllerPressed(wxCommandEvent& event);
    void setPlaygroundPressed(wxCommandEvent& event);
    void donePlaygroundPressed(wxCommandEvent& event);
    void clearPlaygroundPressed(wxCommandEvent& event);
    void setTargetRegionPressed(wxCommandEvent& event);
    void doneTargetRegionPressed(wxCommandEvent& event);
    void clearTargetRegionPressed(wxCommandEvent& event);
    void autoCalcVMaxPressed(wxCommandEvent& event);
    void startSessionPressed(wxCommandEvent& event);
    void pauseSessionPressed(wxCommandEvent& event);
    void stopSessionPressed(wxCommandEvent& event);

    playground_panel_widgets_t playgroundWidgets;
    control_law_dialog_widgets_t controlLawWidgets;

    PlaygroundDisplayWidget* displayWidget;

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_CALIBRATION_PLAYGROUND_PANEL_H
