/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/commands/save_lpm_command.h>
#include <lcmtypes/commands/load_lpm_command.h>
#include <system/module_communicator.h>
#include <utils/command_line.h>
#include <iostream>
#include <random>
#include <string>
#include <cassert>
#include <ctime>

using namespace vulcan;


const std::string kLoad("load");
const std::string kSave("save");
const std::string kMap("map");


void send_save_lpm(const std::string& filename, system::ModuleCommunicator& communicator);
void send_load_lpm_at_origin(const std::string& filename, system::ModuleCommunicator& communicator);


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({kLoad, "Option to load a map", true, ""});
    arguments.push_back({kSave, "Option to save a map", true, ""});
    arguments.push_back({kMap,  "Map to be loaded or saved", false, ""});
    
    utils::CommandLine commandLine(argc, argv, arguments);
    
    if(!commandLine.verify())
    {
        commandLine.printHelp();
        return -1;
    }
    
    system::ModuleCommunicator communicator;
    
    if(commandLine.argumentExists(kSave))
    {
        send_save_lpm(commandLine.argumentValue(kMap), communicator);
    }
    
    if(commandLine.argumentExists(kLoad))
    {
        send_load_lpm_at_origin(commandLine.argumentValue(kMap), communicator);
    }
    
    return 0;
}


void send_save_lpm(const std::string& filename, system::ModuleCommunicator& communicator)
{
    vulcan_lcm::save_lpm_command command;
    command.filename = filename;
    communicator.sendMessage(command);
    
    std::cout << "Sent command to save current LPM to " << filename << '\n';
}


void send_load_lpm_at_origin(const std::string& filename, system::ModuleCommunicator& communicator)
{
    vulcan_lcm::load_lpm_command command;
    command.filename = filename;
    command.initial_x = 0.0f;
    command.initial_y = 0.0f;
    
    communicator.sendMessage(command);
    
    std::cout << "Sent command to load current LPM from " << filename << '\n';
}
