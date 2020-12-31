/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_app.cpp
* \author   Collin Johnson
*
* Definition of LogicalInterfaceApp.
*/

#include <iostream>
#include "utils/config_file.h"
#include "ui/logical/params.h"
#include "ui/logical/logical_interface_app.h"
#include "ui/logical/logical_interface_frame.h"

using namespace vulcan;
using namespace vulcan::ui;

IMPLEMENT_APP(LogicalInterfaceApp)


bool LogicalInterfaceApp::OnInit(void)
{
    wxString configFilename("debug_ui.cfg");
    
    if(argc > 1)
    {
        configFilename = (argv[1]);
    }

    utils::ConfigFile config(configFilename.ToStdString());

    logical_interface_params_t params = load_logical_interface_params(config);

    mainFrame = new LogicalInterfaceFrame(params);

    mainFrame->Show(true);
    SetTopWindow(mainFrame);

    return true;
}


int LogicalInterfaceApp::OnExit(void)
{
    return 0;
}
