/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     main.cpp
* \author   Collin Johnson
*
* Implementation of main function and associated code for running the logplayer.
*/

#include <iostream>
#include "logging/logplayer/log_player.h"
#include "logging/logplayer/params.h"
#include "utils/command_line.h"
#include "utils/config_file.h"

using namespace vulcan;

const std::string HELP_SHORT("h");
const std::string HELP_LONG ("help");
const std::string LOG       ("log");
const std::string FORMAT    ("format");
const std::string CONFIG    ("config-file");
const std::string SPEED     ("speed");

const std::string CARMEN_FORMAT("carmen");
const std::string SAPHIRA_FORMAT("saphira");

const float DEFAULT_SPEED = 1.0f;

void                  display_help_if_needed (const utils::CommandLine& commandLine);
logplayer::log_type_t type_from_format_string(const std::string& format);


/**
* logplayer is a command-line logplayer for Vulcan. This logplayer simply plays back a
* desired log file at a certain speed. No further control is available. For finer-grained
* control over the log playback, use the logplayer-gui (forthcoming).
*
* The command-line options for the logplayer module are:
*
*   -h/--help                   Display help message
*   --log 'filename'            Filename of the log to be played
*   --format 'log type'         File format of the log
*           Currently supported log formats:
*               carmen : CARMEN for playback of Killian Court and all Carmen logs
*               saphira : Saphira for playback of SDR dataset
*   --config-file 'filename'    Filename with configuration for logplayer -- data channels and the like
*   --speed 'value'             Playback speed for the log (>1 = faster, <1 = slower, default = 1.0) (optional)
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    std::string           logFile = commandLine.argumentValue(LOG);
    logplayer::log_type_t format  = type_from_format_string(commandLine.argumentValue(FORMAT));

    utils::ConfigFile              config(commandLine.argumentValue(CONFIG));
    logplayer::log_player_params_t params = logplayer::load_log_player_params(config);

    float speed = DEFAULT_SPEED;

    if(commandLine.argumentExists(SPEED))
    {
        speed = atof(commandLine.argumentValue(SPEED).c_str());
    }

    logplayer::LogPlayer player(params);

    player.setPlaybackSpeed(speed);
    player.load(logFile, format);
    player.play(true);
    player.waitForEnd();

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool needHelp = commandLine.argumentExists(HELP_SHORT) ||
                    commandLine.argumentExists(HELP_LONG)  ||
                    !commandLine.argumentExists(LOG)       ||
                    !commandLine.argumentExists(FORMAT)    ||
                    !commandLine.argumentExists(CONFIG);

    if(needHelp)
    {
        std::cout<<"The command-line options for the logplayer module are:\n"
                 <<'\n'
                 <<"   -h/--help                   Display help message\n"
                 <<"   --log 'filename'            Filename of the log to be played\n"
                 <<"   --format 'log type'         File format of the log\n"
                 <<"           Currently supported log formats:\n"
                 <<"               carmen : CARMEN for playback of Killian Court\n"
                 <<"  --config-file 'filename'    Filename with configuration for logplayer -- data channels and the like\n"
                 <<"   --speed 'value'             Playback speed for the log (>1 = faster, <1 = slower, default = 1.0) (optional)\n";

        exit(-1);
    }
}


logplayer::log_type_t type_from_format_string(const std::string& format)
{
    if(format == CARMEN_FORMAT)
    {
        return logplayer::LOG_CARMEN;
    }
    else if(format == SAPHIRA_FORMAT)
    {
        return logplayer::LOG_SAPHIRA;
    }

    std::cerr<<"WARNING:logplayer: Unknown log format: "<<format<<'\n';

    return logplayer::AUTO_DETECT_LOG_TYPE;
}
