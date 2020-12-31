/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_interface_control.h
* \author   Collin Johnson
*
* Declaration of DecisionInterfaceControl.
*/

#ifndef UI_NAVIGATION_DECISION_INTERFACE_CONTROL_H
#define UI_NAVIGATION_DECISION_INTERFACE_CONTROL_H

#include "ui/common/gl_event.h"
#include "core/pose.h"

namespace vulcan
{
namespace planner { class NavigationInterface; }
namespace system { class SystemCommunicator; }
namespace ui
{

class NavigationData;
class NavigationInterfaceDisplay;

/**
* DecisionInterfaceControl
*
* DecisionInterfaceControl currently supports control of the robot at the Decision level using the keyboard. The
* following commands control the robot's behavior:
*
*   - up/down/left/right : select amongst the path segments incident to a decision point
*   - Ctrl + left/right : cycle through all available actions in the current area
*   - Enter : send the currently selected action to the planner
*
* TODO: Selection of Decisions using the mouse
*/
class DecisionInterfaceControl : public GLKeyboardHandler
{
public:

    /**
    * Constructor for DecisionInterfaceControl.
    *
    * \param    interface           Interface to use for Decision-level navigation
    * \param    display             Display to be updated with Decision-level state
    */
    DecisionInterfaceControl(planner::NavigationInterface& interface, NavigationInterfaceDisplay& display);

    /**
    * update
    */
    void update(const NavigationData& data, system::SystemCommunicator& communicator);

    // GLKeyboardHandler interface
    GLEventStatus keyPressed(wxKeyEvent& key) override;

private:

    planner::NavigationInterface& interface_;
    NavigationInterfaceDisplay& display_;

    pose_t pose_;
    int32_t prevArea_;
    int32_t prevMap_;

    std::size_t hoverIndex_;
    std::size_t selectedIndex_;
    bool didSelectArea_;
};

}
}

#endif // UI_NAVIGATION_DECISION_INTERFACE_CONTROL_H
