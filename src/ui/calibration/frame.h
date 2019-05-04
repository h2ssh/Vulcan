/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     calibration_frame.h
* \author   Collin Johnson
* 
* Declaration of CalibrationFrame.
*/

#ifndef UI_CALIBRATION_CALIBRATION_FRAME_H
#define UI_CALIBRATION_CALIBRATION_FRAME_H

#include <ui/common/ui_main_frame.h>
#include <ui/calibration/calibration_ui.h>

namespace vulcan
{
namespace ui
{
    
class PlaygroundPanel;
class WheelchairPanel;

/**
* CalibrationFrame organizes the initialization of the individual panels that make up the
* CalibrationUI. After the panels are constructed, they are passed off to the UIMainFrame
* base class, which handles the flow of events for the frame.
*/
class CalibrationUIFrame : public CalibrationFrame
{
public:
    
    /**
    * Constructor for CalibrationUIFrame.
    * 
    * \param    params      Parameters for the frame
    */
    CalibrationUIFrame(const calibration_ui_params_t& params);
    
private:
    
    void setupPlaygroundPanel(void);
    void setupWheelchairPanel(void);
    
    PlaygroundPanel* playgroundPanel;
    WheelchairPanel* wheelchairPanel;
};
    
}
}

#endif // UI_CALIBRATION_CALIBRATION_FRAME_H
