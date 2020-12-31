/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/mapeditor/map_editor_app.h"
#include "utils/config_file.h"
#include "ui/common/ui_params.h"
#include "ui/mapeditor/map_editor_frame.h"
#include "hssh/local_topological/params.h"
#include <iostream>


using vulcan::ui::MapEditorApp;


IMPLEMENT_APP(MapEditorApp)


bool MapEditorApp::OnInit(void)
{
    if(argc < 3)
    {
        std::cout << "Using default config files for map_editor: debug_ui.cfg local_topo_hssh.cfg  \n";
    }

    wxString configFilename("debug_ui.cfg");
    wxString localTopoFilename("local_topo_hssh.cfg");
    
    if(argc > 1)
    {
        configFilename = argv[1];
    }
    
    if(argc > 2)
    {
        localTopoFilename = argv[2];
    }
    
    vulcan::utils::ConfigFile config(configFilename.ToStdString());
    vulcan::utils::ConfigFile localTopoConfig(localTopoFilename.ToStdString());
    
    ui_params_t params = load_ui_params(config);
    hssh::local_topology_params_t localTopoParams{localTopoConfig};
    
    mainFrame = new MapEditorFrame(params, localTopoParams);
    
    mainFrame->Show(true);
    SetTopWindow(mainFrame);
    
    return true;
}


int MapEditorApp::OnExit(void)
{
    return 0;
}
