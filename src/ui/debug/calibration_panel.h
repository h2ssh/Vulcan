/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     calibration_panel.h
* \author   Jong Jin Park
*
* Definition of the CalibrationPanel, which provides event handling for the Calibration display tab.
*/

#ifndef UI_DEBUG_CALIBRATION_PANEL_H
#define UI_DEBUG_CALIBRATION_PANEL_H

#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_params.h"
#include "ui/common/ui_panel.h"
#include "core/laser_scan.h"
#include "utils/mutex.h"
#include "sensors/sensor_log.h"
#include <wx/wx.h>
#include <atomic>

namespace vulcan
{
namespace ui
{

class  CalibrationDisplayWidget;

struct calibration_panel_widgets_t
{
    CalibrationDisplayWidget* displayWidget = nullptr;

    // Text boxes for manual calibration
    wxCheckBox* showFrontLaser = nullptr;
    wxCheckBox* showBackLaser = nullptr;
    wxTextCtrl* frontLaserTextX = nullptr;
    wxTextCtrl* frontLaserTextY = nullptr;
    wxTextCtrl* frontLaserTextTheta = nullptr;
    wxTextCtrl* frontLaserTextPitch = nullptr;
    wxTextCtrl* frontLaserTextRoll = nullptr;
    wxTextCtrl* backLaserTextX = nullptr;
    wxTextCtrl* backLaserTextY = nullptr;
    wxTextCtrl* backLaserTextTheta = nullptr;
    wxTextCtrl* backLaserTextPitch = nullptr;
    wxTextCtrl* backLaserTextRoll = nullptr;

    // Text boxes for automatic tilt calibration
    wxRadioBox* tiltLaserToShowRadio = nullptr;
    wxTextCtrl* tiltStartIndex = nullptr;
    wxTextCtrl* tiltEndIndex = nullptr;
    wxButton*   calibratePitchButton = nullptr;
    wxButton*   calibrateRollButton = nullptr;
};

/**
* CalibrationPanel is responsible for event handling in the calibration panel.
*
* The CalibrationPanel currently supports the following modes:
*
*   - manual : manual mode is entered as soon as laser data arrives via LCM. In manual mode, the user types in the
*       (x, y, theta) calibration between laser and robot frames. The data is initially seeded with the stored offset
*       in the laser data.
*
*   - tilt : tilt mode is entered as soon as tilt data is loaded from a sensor log.
*/
class CalibrationPanel : public UIPanel
{
public:

    /**
    * Constructor for CalibrationPanel.
    *
    * \param    params          Parameters to the panel and widget
    * \param    widget          CalibrationDisplayWidget instance to be controlled by the panel
    */
    CalibrationPanel(const ui_params_t& params, const calibration_panel_widgets_t& widgets);

    // UIPanel interface
    void setup       (wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe   (system::ModuleCommunicator& producer) override;
    void setConsumer (system::ModuleCommunicator* consumer) override;
    void update      (void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

    // Data handlers
    void handleData(const polar_laser_scan_t& scan, const std::string& channel);

private:

    using index_range = std::pair<int, int>;

    // Which calibration is being performed?
    enum Mode
    {
        manual,
        tilt,
    };

    enum TiltLaser
    {
        front,
        back,
    };

    Mode mode_;
    std::atomic<Mode> newMode_;      // mode to apply on next iteration, based on new inputs
    calibration_panel_widgets_t widgets;

    system::ModuleCommunicator* consumer;

    pose_6dof_t frontLaserPoseInRobotFrame;
    pose_6dof_t backLaserPoseInRobotFrame;

    // Flag indicating if the front and back laser poses should be auto-filled with the next piece of laser data
    bool haveFrontLaserPose_;
    bool haveBackLaserPose_;

    polar_laser_scan_t frontScan_;
    polar_laser_scan_t backScan_;

    sensors::SensorLog tiltData_;
    index_range tiltRange_;
    TiltLaser tiltLaser_;

    utils::Mutex laserLock_;

    calibration_display_params_t params;

    // State machine functionality
    void enterManualMode(void);
    void updateManualMode(void);
    void startManualMode(void);
    void stopManualMode(void);
    void toggleManualWidgets(bool enable);

    void enterTiltMode(void);
    void updateTiltMode(void);
    void startTiltMode(void);
    void stopTiltMode(void);
    void toggleTiltWidgets(bool enable);

    void calibrateTilt(sensors::SensorLog::laser_iterator begin, sensors::SensorLog::laser_iterator end, bool doPitch);
    void setTiltLines(polar_laser_scan_t& lineScan, polar_laser_scan_t& notLineScan);

    // Event handlers
    void showFrontLaserChecked(wxCommandEvent& event);
    void showBackLaserChecked (wxCommandEvent& event);
    void setFrontLaserX       (wxCommandEvent& event);
    void setFrontLaserY       (wxCommandEvent& event);
    void setFrontLaserTheta   (wxCommandEvent& event);
    void setFrontLaserPitch   (wxCommandEvent& event);
    void setFrontLaserRoll    (wxCommandEvent& event);
    void setBackLaserX        (wxCommandEvent& event);
    void setBackLaserY        (wxCommandEvent& event);
    void setBackLaserTheta    (wxCommandEvent& event);
    void setBackLaserPitch    (wxCommandEvent& event);
    void setBackLaserRoll     (wxCommandEvent& event);

    void loadTiltDataPressed(wxCommandEvent& event);
    void tiltLaserChanged(wxCommandEvent& event);
    void lineStartIndexChanged(wxCommandEvent& event);
    void lineEndIndexChanged(wxCommandEvent& event);
    void calibratePitchPressed(wxCommandEvent& event);
    void calibrateRollPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_METRIC_PLANNER_PANEL_H
