/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cstdlib>
#include <string>
#include <iostream>
#include "utils/command_line.h"
#include "utils/thread.h"
#include "utils/timestamp.h"
#include "robot/quantum_6000.h"


using vulcan::robot::Quantum6000;


const std::string HELP_SHORT("h");
const std::string HELP_LONG("help");
const std::string BUS_A_PORT("bus-a-port");
const std::string BUS_B_PORT("bus-b-port");


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine);


int main(int argc, char** argv)
{
    vulcan::utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    vulcan::utils::Thread quantumThread;

    Quantum6000 quantum(commandLine.argumentValue(BUS_A_PORT),
                        commandLine.argumentValue(BUS_B_PORT));


    quantumThread.attachTask(&quantum);

    quantumThread.start();
    
    usleep(5000000);

    for(int n = 0; n < 3; ++n)
    {
        for(int i = 0; i < 300; ++i)
        {
            vulcan::robot::joystick_command_t go;
            go.timestamp = vulcan::utils::system_time_us();
            go.gain = 100;
            go.forward = 30;
            go.left = 0;

            quantum.setJoystickCommand(go);
            usleep(10000);
        }

        vulcan::robot::joystick_command_t stop;
        stop.timestamp = vulcan::utils::system_time_us();
        stop.gain = 100;
        stop.forward = 0;
        stop.left = 0;

        quantum.setJoystickCommand(stop);
        usleep(2000000);
    }
    
    // Currently only testing that the passthrough is working
    char dummy;
    std::cin>>dummy;
    std::cout << "Entered:" << dummy << std::endl;

    quantumThread.kill();

    return 0;
}


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine)
{
    bool helpNeeded = commandLine.argumentExists(HELP_SHORT) ||
                      commandLine.argumentExists(HELP_LONG)  ||
                      !commandLine.argumentExists(BUS_A_PORT) ||
                      !commandLine.argumentExists(BUS_B_PORT);

    if(helpNeeded)
    {
        exit(1);
    }

}
