/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_NAVIGATION_NAVIGATION_INTERFACE_APP_H
#define UI_NAVIGATION_NAVIGATION_INTERFACE_APP_H

#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class NavigationInterfaceFrame;

/**
 * NavigationInterfaceApp is where the main
 */
class NavigationInterfaceApp : public wxApp
{
public:
    virtual bool OnInit(void);
    virtual int OnExit(void);

private:
    NavigationInterfaceFrame* mainFrame;
};

DECLARE_APP(NavigationInterfaceApp)

}   // namespace ui
}   // namespace vulcan

#endif   // UI_NAVIGATION_NAVIGATION_INTERFACE_APP_H
