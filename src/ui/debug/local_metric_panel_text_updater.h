/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_panel_text_updater.h
* \author   Collin Johnson
*
* Declaration of LocalMetricPanelTextUpdater.
*/

#ifndef UI_DEBUG_LPM_PANEL_TEXT_UPDATER_H
#define UI_DEBUG_LPM_PANEL_TEXT_UPDATER_H

#include "ui/common/ui_forward_declarations.h"
#include "utils/mutex.h"
#include <wx/string.h>
#include <string>

class wxStaticText;

namespace vulcan
{
namespace ui
{

struct local_metric_panel_text_widgets_t
{
    wxStaticText* pose;
    wxStaticText* velocity;
    wxStaticText* command;
    wxStaticText* imuAccel;
    wxStaticText* imuVelocity;
    wxStaticText* imuOrientation;
    wxStaticText* leftWheel;
    wxStaticText* rightWheel;
    
    local_metric_panel_text_widgets_t(void)
    : pose(0)
    , velocity(0)
    , command(0)
    , imuAccel(0)
    , imuVelocity(0)
    , imuOrientation(0)
    , leftWheel(0)
    , rightWheel(0)
    {
    }
};

/**
* LocalMetricPanelTextUpdater takes LPM and sensor data that is displayed in text form, and
* updates the displayed text when new data arrives.
*/
class LocalMetricPanelTextUpdater
{
public:

    LocalMetricPanelTextUpdater(const local_metric_panel_text_widgets_t& widgets);

    void update(void);

    virtual void handleData(const imu_data_t&         imuData,  const std::string& channel);
    virtual void handleData(const motion_state_t&       motion,   const std::string& channel);
    virtual void handleData(const robot::commanded_velocity_t& command,  const std::string& channel);

private:

    wxString poseString_;
    wxString velocityString_;
    wxString commandString_;
    wxString imuAccelString_;
    wxString imuVelocityString_;
    wxString imuOrientationString_;
    wxString leftWheelString_;
    wxString rightWheelString_;

    local_metric_panel_text_widgets_t textWidgets_;

    utils::Mutex displayLock_;
};

}
}

#endif // UI_DEBUG_LPM_PANEL_TEXT_UPDATER_H
