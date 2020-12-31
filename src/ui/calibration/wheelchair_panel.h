/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     wheelchair_panel.h
 * \author   Collin Johnson
 *
 * Declaration of WheelchairPanel.
 */

#ifndef UI_CALIBRATION_WHEELCHAIR_PANEL_H
#define UI_CALIBRATION_WHEELCHAIR_PANEL_H

namespace vulcan
{
namespace ui
{

#include "ui/common/ui_panel.h"

/**
 * WheelchairPanel
 */
class WheelchairPanel : public UIPanel
{
public:
    // UIPanel interface
    virtual void setup(wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe(system::ModuleCommunicator& producer);
    virtual void setConsumer(system::ModuleCommunicator* consumer);
    virtual void update(void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

private:
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_CALIBRATION_WHEELCHAIR_PANEL_H
