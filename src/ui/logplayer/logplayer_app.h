/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logplayer_app.h
* \author   Collin Johnson
*
* Declaration of LogplayerApp, which handles the creation of the Logplayer UI.
*/

#ifndef UI_LOGPLAYER_LOGPLAYER_APP_H
#define UI_LOGPLAYER_LOGPLAYER_APP_H

#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class LogplayerFrame;

class LogplayerApp : public wxApp
{
public:

    virtual bool OnInit(void);
    virtual int  OnExit(void);

private:

    LogplayerFrame* mainFrame;
};

DECLARE_APP(LogplayerApp)

}
}

#endif // UI_LOGPLAYER_LOGPLAYER_APP_H
