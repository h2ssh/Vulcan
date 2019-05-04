/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/commands/direct_control_command.h>
#include <utils/timestamp.h>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    const char kQuit = 'q';
    const char kStraight = 'w';
    const char kStop = 's';
    const char kLeft = 'a';
    const char kRight = 'd';
    const std::string kSource("keyboard_control");

    std::cout << "Simple keyboard control of Vulcan via the AbsoluteMotionTask in the control_planner:\n"
        << "Straight:   " << kStraight << '\n'
        << "Stop:       " << kStop << '\n'
        << "Turn left:  " << kLeft << '\n'
        << "Turn right: " << kRight << '\n'
        << "Quit:       " << kQuit << '\n';

    lcm::LCM transmitter;

    char input = 0;

    while(input != kQuit)
    {
        std::cin >> input;
        std::cin.ignore(1);

        vulcan_lcm::direct_control_command command;
        command.timestamp = vulcan::utils::system_time_us();
        command.source = kSource;

        switch(input)
        {
            case kQuit:
                std::cout << "Exiting program. Good-bye!\n";
                command.command = vulcan_lcm::direct_control_command::STOP;
                break;

            case kStraight:
                std::cout << "Going straight\n";
                command.command = vulcan_lcm::direct_control_command::GO_STRAIGHT;
                break;

            case kStop:
                std::cout << "Stopping\n";
                command.command = vulcan_lcm::direct_control_command::STOP;
                break;

            case kLeft:
                std::cout << "Turning left\n";
                command.command = vulcan_lcm::direct_control_command::TURN_LEFT;
                break;

            case kRight:
                std::cout << "Turning right\n";
                command.command = vulcan_lcm::direct_control_command::TURN_RIGHT;
                break;

            default:
                std::cout << "ERROR: Unknown command:" << input << '\n';
        }

        transmitter.publish("DIRECT_CONTROL_COMMAND", &command);
    }

    return 0;
}
