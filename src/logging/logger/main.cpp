/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "robot/commands.h"
#include "core/motion_state.h"
#include "sensors/imu.h"
#include "core/odometry.h"
#include "system/module_communicator.h"
#include "utils/command_line.h"
#include "utils/repeated_task.h"
#include "logging/logger/data_logger.h"
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace vulcan;

const std::string HELP_SHORT("h");
const std::string HELP_LONG ("help");
const std::string BASENAME  ("basename");


void display_help_if_needed(const utils::CommandLine& commandLine);
//make sure
//1. comment out the unwanted info

int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);
    display_help_if_needed(commandLine);

    std::string basename = commandLine.argumentValue(BASENAME);
    logging::DataLogger logger(basename);

    system::ModuleCommunicator communicator;
    communicator.subscribeTo<motion_state_t>(&logger);
//     communicator.subscribeTo<tracker::DynamicObjectCollection>(&logger);
    communicator.subscribeTo<imu_data_t>(&logger);
    communicator.subscribeTo<encoder_data_t>(&logger);
    communicator.subscribeTo<robot::motion_command_t>(&logger);
    communicator.subscribeTo<robot::commanded_joystick_t>(&logger);
//communicator.subscribeTo<planner::trajectory_planner_info_t>(&logger);
//     communicator.subscribeTo<polar_laser_scan_t>(&logger);
//     communicator.subscribeTo<polar_laser_scan_t>(&logger);
//     communicator.subscribeTo<hssh::LocalPerceptualMap>(&logger);

    std::cout<<"Beginning to log data...\n";
    std::cout<<"Press 'ENTER' to stop program\n";
    sleep(2);

    auto receiverFunc = [&communicator](bool killed) -> bool { communicator.processIncoming(); return !killed; };
    utils::RepeatedTask receiverTask(receiverFunc);

    std::cin.get();
    
    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool needHelp = commandLine.argumentExists(HELP_SHORT) ||
                    commandLine.argumentExists(HELP_LONG)  ||
                    !commandLine.argumentExists(BASENAME);

    if(needHelp)
    {
        std::cout<<"The command-line options for the logplayer module are:\n"
                 <<'\n'
                 <<"   -h/--help                        Display help message\n"
                 <<"   --basename 'filename'   Filename of the log to be stored\n";

        exit(-1);
    }
}
