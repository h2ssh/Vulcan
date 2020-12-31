/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     navigation_interface_frame.cpp
 * \author   Collin Johnson
 *
 * Definition of NavigationInterfaceFrame.
 */

#include "ui/navigation/navigation_interface_frame.h"
#include "ui/navigation/goal_interface_control.h"
#include "ui/navigation/navigation_interface_control.h"
#include "ui/navigation/navigation_interface_display.h"

namespace vulcan
{
namespace ui
{

NavigationInterfaceFrame::NavigationInterfaceFrame(void) : NavigationInterface(0)
{
    display->pushKeyboardHandler(this);
    setupNavigationDisplay();
    initialize(nullptr, 30, display, nullptr);
}


NavigationInterfaceFrame::~NavigationInterfaceFrame(void)
{
    // For std::unique_ptr
}


GLEventStatus NavigationInterfaceFrame::keyReleased(wxKeyEvent& key)
{
    if (key.GetKeyCode() == 'F') {
        ShowFullScreen(!IsFullScreen());
    }

    return GLEventStatus::passthrough;
}


void NavigationInterfaceFrame::setupNavigationDisplay(void)
{
    GoalInterfaceWidgets goalWidgets;
    goalWidgets.goalsList = namedGoalsList;
    goalWidgets.selectGoalButton = selectGoalButton;
    goalWidgets.addSelectedButton = addSelectedButton;
    goalWidgets.addCurrentButton = addCurrentLocationButton;
    goalWidgets.previewButton = previewRouteButton;
    goalWidgets.goButton = goButton;

    navigationControl_ = new NavigationInterfaceControl(display, goalWidgets);
    addPanel(navigationControl_, nullptr);
}

}   // namespace ui
}   // namespace vulcan
