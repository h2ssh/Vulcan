/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     logplayer_app.cpp
 * \author   Collin Johnson
 *
 * Definition of LogplayerApp.
 */

#include "ui/logplayer/logplayer_app.h"
#include "logging/logplayer/log_player.h"
#include "logging/logplayer/params.h"
#include "ui/logplayer/logplayer_frame.h"
#include "utils/config_file.h"

using namespace vulcan;

IMPLEMENT_APP(ui::LogplayerApp)

bool ui::LogplayerApp::OnInit(void)
{
    wxString configFilename(argv[1]);

    utils::ConfigFile config(std::string(configFilename.mb_str()));
    logplayer::log_player_params_t params = logplayer::load_log_player_params(config);

    std::unique_ptr<logplayer::LogPlayer> player(new logplayer::LogPlayer(params));
    mainFrame = new ui::LogplayerFrame(std::move(player));

    mainFrame->Show(true);
    SetTopWindow(mainFrame);

    return true;
}


int ui::LogplayerApp::OnExit(void)
{
    return 0;
}
