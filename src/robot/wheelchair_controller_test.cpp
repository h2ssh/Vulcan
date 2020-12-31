/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "system/module_communicator.h"
#include "robot/commands.h"
#include "utils/command_line.h"
#include "utils/timestamp.h"
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>


const std::string HELP_SHORT("h");
const std::string HELP_LONG("help");
const std::string CHANNEL("channel");
const std::string LINEAR("linear");
const std::string ANGULAR("angular");
const std::string DURATION("duration");
const std::string ITERATIONS("iterations");


struct square_wave_t
{
    float   linear;
    float   angular;
    int64_t duration;
    uint8_t iterations;
};


void          display_help_if_needed(const vulcan::utils::CommandLine& commandLine);
square_wave_t load_square_wave(const vulcan::utils::CommandLine& commandLine);
void          run_square_wave(const square_wave_t& wave, vulcan::system::ModuleCommunicator& transmitter, const std::string& channel);

/**
* wheelchair_controller_test is a simple program that tests the controller by sending a square wave of motor
* commands. The command-line arguments control the velocity and duration of the square wave.
*
* The command-line arguments for the wheelchair_controller_test are:
*
*   --help/-h               Display this message
*   --channel 'name'        Name of the channel on which to publish the motion commands
*   --linear  'm/s'         Linear velocity of the motion command  [0, 1.5]
*   --angular 'rad/s'       Angular velocity of the motion coomand [-2, 2]
*   --duration 'seconds'    Number of seconds to issue the command for
*   --iterations 'number'   Number of iterations of the square wave to run
*/
int main(int argc, char** argv)
{
    vulcan::utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    std::string   channel = commandLine.argumentValue(CHANNEL);
    square_wave_t wave    = load_square_wave(commandLine);

    vulcan::system::ModuleCommunicator transmitter;

    run_square_wave(wave, transmitter, channel);

    return 0;
}


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine)
{
    bool helpNeeded = commandLine.argumentExists(HELP_SHORT) ||
                      commandLine.argumentExists(HELP_LONG)  ||
                      !commandLine.argumentExists(CHANNEL)   ||
                      !commandLine.argumentExists(LINEAR)    ||
                      !commandLine.argumentExists(ANGULAR)   ||
                      !commandLine.argumentExists(DURATION)  ||
                      !commandLine.argumentExists(ITERATIONS);
    if(helpNeeded)
    {
        std::cout<<"The command-line arguments for the wheelchair_controller_test are:\n"
                 <<'\n'
                 <<"--help/-h               Display this message\n"
                 <<"--channel 'name'        Name of the channel on which to publish the motion commands\n"
                 <<"--linear  'm/s'         Linear velocity of the motion command  [0, 1.5]\n"
                 <<"--angular 'rad/s'       Angular velocity of the motion command [-2, 2]\n"
                 <<"--duration 'seconds'    Number of seconds to issue the command for\n"
                 <<"--iterations 'number'   Number of iterations of the square wave to run\n";
        exit(1);
    }
}


square_wave_t load_square_wave(const vulcan::utils::CommandLine& commandLine)
{
    square_wave_t wave;

    wave.linear     = atof(commandLine.argumentValue(LINEAR).c_str());
    wave.angular    = atof(commandLine.argumentValue(ANGULAR).c_str());
    wave.duration   = atoi(commandLine.argumentValue(DURATION).c_str()) * 1000000;
    wave.iterations = atoi(commandLine.argumentValue(ITERATIONS).c_str());

    return wave;
}


void run_square_wave(const square_wave_t& wave, vulcan::system::ModuleCommunicator& transmitter, const std::string& channel)
{
    std::cout<<"Running square wave with: ("<<wave.linear<<','<<wave.angular<<"), Duration="<<(wave.duration/1000000)<<" seconds, Iterations="<<wave.iterations<<'\n';

    int64_t startTime = 0;

    vulcan::robot::velocity_command_t commandToSend;
//     commandToSend.linearVelocity  = wave.velocity;
//     commandToSend.angularVelocity = 0;

    commandToSend.linearVelocity  = wave.linear;
    commandToSend.angularVelocity = wave.angular;

    for(int n = 0; n < wave.iterations; ++n)
    {
        std::cout<<"Running iteration:"<<n<<'\n';

        startTime = vulcan::utils::system_time_us();

        std::cout<<"Sending velocity:("<<wave.linear<<','<<wave.angular<<")\n";

        while(vulcan::utils::system_time_us() - startTime < wave.duration)
        {
            commandToSend.timestamp       = vulcan::utils::system_time_us();
            commandToSend.linearVelocity  = wave.linear;
            commandToSend.angularVelocity = wave.angular;

            transmitter.sendMessage(commandToSend, channel);

            usleep(100000);
        }

        startTime = vulcan::utils::system_time_us();

        std::cout<<"Sending velocity:0\n";

        while(vulcan::utils::system_time_us() - startTime < wave.duration)
        {
            commandToSend.timestamp       = vulcan::utils::system_time_us();
            commandToSend.angularVelocity = 0;
            commandToSend.linearVelocity  = 0;

            transmitter.sendMessage(commandToSend, channel);

            usleep(100000);
        }
    }
}
