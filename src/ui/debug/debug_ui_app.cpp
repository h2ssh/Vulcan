/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/debug/debug_ui_app.h"
#include "ui/common/ui_params.h"
#include "ui/debug/debug_ui_frame.h"
#include "utils/config_file.h"


using vulcan::ui::DebugUIApp;


IMPLEMENT_APP(DebugUIApp)


bool DebugUIApp::OnInit(void)
{
    if (argv < 2) {
        std::cout << "Using default config file for debug_ui:  debug_ui.cfg\n";
    }

    wxString configFilename("debug_ui.cfg");

    if (argv > 1) {
        configFilename = argv[1];
    }

    vulcan::utils::ConfigFile config(configFilename.ToStdString());
    mainFrame = new DebugUIFrame(config);

    mainFrame->Show(true);
    SetTopWindow(mainFrame);

    return true;
}


int DebugUIApp::OnExit(void)
{
    return 0;
}
