/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/simulator/simulator_app.h"
#include "ui/simulator/simulator_frame.h"


using vulcan::ui::SimulatorApp;


IMPLEMENT_APP(SimulatorApp)


bool SimulatorApp::OnInit(void)
{
    mainFrame = new SimulatorFrame();

    mainFrame->Show(true);
    SetTopWindow(mainFrame);

    return true;
}


int SimulatorApp::OnExit(void)
{
    return 0;
}
