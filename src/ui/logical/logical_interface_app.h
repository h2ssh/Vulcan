/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_app.h
* \author   Collin Johnson
*
* Declaration of LogicalInterfaceApp, which launches the HSSH Logical Interface.
*/

#ifndef UI_LOGICAL_LOGICAL_INTERFACE_APP_H
#define UI_LOGICAL_LOGICAL_INTERFACE_APP_H

#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class LogicalInterfaceFrame;

/**
* LogicalInterfaceApp is where the main function for the logical interface UI is implemented, per
* the wxWidgets way of doing things.
*/
class LogicalInterfaceApp : public wxApp
{
public:

    virtual bool OnInit(void);
    virtual int  OnExit(void);

private:

    LogicalInterfaceFrame*   mainFrame;
};

DECLARE_APP(LogicalInterfaceApp)

}
}

#endif // UI_LOGICAL_LOGICAL_INTERFACE_APP_H
