/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     calibration_display_widget.cpp
* \author   Jong Jin Park
* 
* Implemenation of CalibrationDisplayWidget.
*/

#include <ui/debug/calibration_display_widget.h>
#include <ui/common/default_colors.h>
#include <ui/common/ui_params.h>
#include <ui/components/laser_scan_renderer.h>
#include <ui/components/corner_renderer.h>
#include <core/laser_scan.h>
#include <utils/auto_mutex.h>

namespace vulcan
{
namespace ui
{
  
    
CalibrationDisplayWidget::CalibrationDisplayWidget(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long style,
                                                   const wxString& name,
                                                   const wxPalette& palette)
: OpenGLWidget(parent, id, pos, size, style, name, palette)
, shouldShowFrontLaserScans(true)
, shouldShowBackLaserScans(true)
, laserRenderer(new LaserScanRenderer())
{
}

CalibrationDisplayWidget::~CalibrationDisplayWidget(void)
{
}


void CalibrationDisplayWidget::setWidgetParams(const ui_params_t& params)
{
    
}


void CalibrationDisplayWidget::renderWidget(void)
{
    if(shouldShowFrontLaserScans)
    {
        laserRenderer->setRenderColors(path_color(), path_color());
        laserRenderer->renderScan(frontScan_, frontLaserPoseInRobotFrame, pose_t());
    }
        
    if(shouldShowBackLaserScans)
    {
        laserRenderer->setRenderColors(destination_color(), destination_color());
        laserRenderer->renderScan(backScan_, backLaserPoseInRobotFrame, pose_t());
    }
}
    
} // namespace ui
} // namespace vulcan
