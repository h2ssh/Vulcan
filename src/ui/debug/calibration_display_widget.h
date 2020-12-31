/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     calibration_display_widget.h
 * \author   Jong Jin Park
 *
 * Definition of CalibrationDisplayWidget for rendering LaserCalibarator (tentative name) state.
 */

#ifndef UI_DEBUG_CALIBRATION_DISPLAY_WIDGET_H
#define UI_DEBUG_CALIBRATION_DISPLAY_WIDGET_H

#include "core/laser_scan.h"
#include "core/pose.h"
#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_params.h"
#include "ui/components/open_gl_widget.h"
#include "utils/mutex.h"
#include <deque>
#include <wx/glcanvas.h>
#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class LaserScanRenderer;

/**
 * CalibrationDisplayWidget is a widget that displays the state information generated
 * by the LaserCalibration module using OpenGL.
 *
 * The various data to display can be turned on/off using show/hideXXXX methods.
 */
class CalibrationDisplayWidget : public OpenGLWidget
{
public:
    CalibrationDisplayWidget(wxWindow* parent,
                             wxWindowID id = wxID_ANY,
                             const wxPoint& pos = wxDefaultPosition,
                             const wxSize& size = wxDefaultSize,
                             long style = 0,
                             const wxString& name = wxString((const wxChar*)("GLCanvas")),
                             const wxPalette& palette = wxNullPalette);

    virtual ~CalibrationDisplayWidget(void);

    void setWidgetParams(const ui_params_t& params);

    // laser display
    void showFrontLaserScans(bool show) { shouldShowFrontLaserScans = show; };
    void showBackLaserScans(bool show) { shouldShowBackLaserScans = show; };
    void setFrontLaserPoseInRobotFrame(const pose_6dof_t& laserPoseInRobotFrame)
    {
        frontLaserPoseInRobotFrame = laserPoseInRobotFrame;
    };
    void setBackLaserPoseInRobotFrame(const pose_6dof_t& laserPoseInRobotFrame)
    {
        backLaserPoseInRobotFrame = laserPoseInRobotFrame;
    };

    // NOTE: Not thread-safe!
    void setFrontLaser(const polar_laser_scan_t& scan) { frontScan_ = scan; }
    void setBackLaser(const polar_laser_scan_t& scan) { backScan_ = scan; }

private:
    bool shouldShowFrontLaserScans;
    bool shouldShowBackLaserScans;

    polar_laser_scan_t frontScan_;
    polar_laser_scan_t backScan_;

    pose_6dof_t frontLaserPoseInRobotFrame;
    pose_6dof_t backLaserPoseInRobotFrame;

    std::unique_ptr<LaserScanRenderer> laserRenderer;

    // OpenGLWidget interface
    virtual void renderWidget(void);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_LPM_DISPLAY_WIDGET_H
