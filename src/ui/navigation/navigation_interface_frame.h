/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface_frame.h
* \author   Collin Johnson
* 
* Declaration of NavigationInterfaceFrame.
*/

#ifndef UI_NAVIGATION_NAVIGATION_INTERFACE_FRAME_H
#define UI_NAVIGATION_NAVIGATION_INTERFACE_FRAME_H

#include "ui/navigation/navigation_interface.h"
#include "ui/common/gl_event.h"

namespace vulcan
{
namespace ui
{
    
class NavigationInterfaceControl;

/**
* NavigationInterfaceFrame coordinates the Decision and Goal levels.
* 
* The following keyboard commands control the displayed frame:
* 
*   - F : full screen
*/
class NavigationInterfaceFrame : public NavigationInterface,
                                 public GLKeyboardHandler
{
public:
    
    NavigationInterfaceFrame(void);
    virtual ~NavigationInterfaceFrame(void);
    
    // Handler for keyboard events
    GLEventStatus keyReleased(wxKeyEvent& event) override;

private:
    
    void setupNavigationDisplay(void);

    NavigationInterfaceControl* navigationControl_;
};

}
}

#endif // UI_NAVIGATION_NAVIGATION_INTERFACE_FRAME_H
