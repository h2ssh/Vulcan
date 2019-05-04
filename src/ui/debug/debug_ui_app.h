/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_DEBUG_DEBUG_UI_APP_H
#define UI_DEBUG_DEBUG_UI_APP_H

#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class DebugUIFrame;

/**
* DebugUIApp is where the main  
*/
class DebugUIApp : public wxApp
{
public:
    
    virtual bool OnInit(void);
    virtual int  OnExit(void);
    
private:
    
    DebugUIFrame* mainFrame;
};

DECLARE_APP(DebugUIApp)

}
}

#endif // UI_DEBUG_DEBUG_UI_APP_H
