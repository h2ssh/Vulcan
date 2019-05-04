/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     quantum_6000_calibration_maneuvers.cpp
* \author   Jong Jin Park
*
* Data collecting maneuvers for a quantum 6000 wheelchair with imu and encoders
*/

#include <system/module_communicator.h>
#include <robot/commands.h>
#include <utils/timestamp.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace vulcan;

const int64_t SAMPLING_INTERVAL_US = 60000;
const float   SAMPLING_INTERVAL_S  = 0.06;
// (50000us = 0.05s => 20Hz)

struct test_configuration_t
{
    test_configuration_t(system::ModuleCommunicator& transmitter,
                         int                         numSamples,
                         robot::joystick_command_t&  command)
    : transmitter(transmitter)
    , numSamples(numSamples)
    , command(command)
    {
    }

    system::ModuleCommunicator& transmitter;
    int                         numSamples;
    robot::joystick_command_t   command;
};

void run_test(test_configuration_t& config);
void run_maneuvers(void);

/**
* 2/22/2016 quantum_6000_calibration_maneuvers is used to send out pre-specified joystick commands for robot calibration.
*
* The maneuvers consists of long sequence of pre-determined step tests, random steps and chirp tests;
*/

int main(int argc, char** argv)
{
    std::cout<<"..\n";
    sleep(1);
    
    std::cout<<"Initiating the calibration routine.\n";
    std::cout<<"Switch over to robot control. Waiting 5 seconds...\n";
    sleep(5);
    
    run_maneuvers();

    return 0;
}


void run_test(test_configuration_t& config)
{
    std::cout<<"INFO: Starting test: Command:("<<config.command.forward<<','<<config.command.left<<") Samples:"<<config.numSamples<<'\n';

    for(int n = config.numSamples; --n >= 0;)
    {
        config.command.timestamp = utils::system_time_us();
        config.transmitter.sendMessage(robot::motion_command_t(robot::AUTONOMOUS_CONTROLLER, config.command));
        
        // hold for desired microseconds (50000us = 0.05s => 20Hz)
        usleep(SAMPLING_INTERVAL_US);
    }
}


void run_maneuvers(void)
{
    std::cout<<"This is a test to evaluate the behavior of the wheelchair under changing command inputs."
             <<"\n";

    // initializations
    int   i = 0;
    int   j = 0;
    float sleepTime = 3.0;

    system::ModuleCommunicator transmitter;
    int numSamples = static_cast<int>(1.0/SAMPLING_INTERVAL_S);
    robot::joystick_command_t command(0, 0, 100);
    
    // start at neutral position and hold for 1 second
    test_configuration_t config(transmitter, numSamples, command);
    run_test(config);
    
    
    ////////////////////////////////////////////////////////////////////////////
    // run step tests (alternates between larger and smaller step inputs)
    config.numSamples = 70;   // duration of each commands in number of samples (3.5 sec per step)
    for(i = 0; i < 50; i += 7)
    {
        for(j = 49; j > -50; j -= 7)
        {
            // larger step commands
            config.command.forward = i;
            config.command.left    = j;
            
            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
            
            run_test(config);
            
            // smaller step commands
            config.command.forward = i*2/7;
            if(j == 0)
            {
                config.command.left = 0;
            }
            else
            {
                config.command.left = static_cast<int16_t>(copysign(2*(7 - abs(j)/7), j));
            }
            
            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
            
            run_test(config);
        }
        
    // Go back to the neural position for a few second.
    std::cout<<"INFO: Finished " << (i+7)/7 << "/8 of the sweeps for step inputs...\n"
             <<std::endl;
    
    config.numSamples = static_cast<int>(sleepTime/SAMPLING_INTERVAL_S);
    config.command.forward = 0;
    config.command.left = 0;
    run_test(config);
    
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // additional slow-turn-straight line runs
    for(i = 20; i < 61; i += 10)
    {
        for(j = 10; j > -11; j -= 20)
        {
            // slight turn
            config.numSamples     = 50;
            config.command.forward = i/4;
            config.command.left    = j;
            
            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
            std::cout<<"WARNING!!: The wheelchair will be driving in straight line in approximately 2.5 seconds. Please make sure to have a clear path.\n";
            
            run_test(config);
            
            // straight lines
            config.numSamples     = 70;
            config.command.forward = i;
            config.command.left    = 0;
            
            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
            
            run_test(config);
        }
        
    // Go back to the neural position for a few second.
    std::cout<<"INFO: Finished " << (i-10)/10 << "/5 of the sweeps for turn and drive tests...\n"
             <<std::endl;
    
    config.numSamples = static_cast<int>(sleepTime/SAMPLING_INTERVAL_S);
    config.command.forward = 0;
    config.command.left    = 0;
    run_test(config);
    
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // run tests with sinusoidal inputs
    config.numSamples = 1;     // duration of each commands in number of samples
    for(i = 0; i < 200; i += 1) // 10 sec each for a sinusoid
    {
        // specify commands
        config.command.forward = 20.0*sin(2.0*3.14/100.0*i + 3.14) + 30.0; //  10 < y < 50
        config.command.left    = 40.0*sin(2.0*3.14/200.0*i + 3.14/4.0);    // -40 < x < 40
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 20.0*sin(2.0*3.14/100.0*i + 3.14/4.0) + 30.0; //  10 < y < 50
        config.command.left    = 40.0*sin(2.0*3.14/200.0*i + 3.14/4.0);        // -40 < x < 40
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 10.0*sin(2.0*3.14/100.0*i + 2*3.14/4.0) + 20.0; //  10 < x < 30
        config.command.left    = 30.0*sin(2.0*3.14/200.0*i + 3.14/4.0);          // -30 < x < 30
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 10.0*sin(2.0*3.14/100.0*i + 3*3.14/4.0) + 20.0; //  10 < y < 30
        config.command.left    = 30.0*sin(2.0*3.14/200.0*i + 3.14/4.0);          // -30 < x < 30
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    // Go back to the neural position for a few second.
    std::cout<<"INFO: Finished test runs for the sinusoidal inputs...\n"
             <<"INFO: Continuing to random inputs...\n"
             <<std::endl;
    
    config.numSamples = static_cast<int>(sleepTime/SAMPLING_INTERVAL_S);
    config.command.forward = 0;
    config.command.left    = 0;
    run_test(config);
    
    
    ///////////////////////////////////////////////////////////////////////////
    // run random test with quickly varying inputs
    config.numSamples = 10;    // duration of each commands in number of samples (0.5 sec per step)     
    for(i = 0; i < 40; i += 1)
    {
        // specify commands
        config.command.forward = rand() % 40 + 11;   //  11 < y < 50
        config.command.left    = (rand() % 81) - 40; // -40 < x < 40
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    // Go back to the neural position for a few second.
    std::cout<<"INFO: Finished test runs for the short random inputs...\n"
             <<"INFO: Continuing to long random inputs...\n"
             <<std::endl;
    
    config.numSamples = static_cast<int>(sleepTime/SAMPLING_INTERVAL_S);
    config.command.forward = 0;
    config.command.left    = 0;
    run_test(config);
    
    
    ///////////////////////////////////////////////////////////////////////////
    // run random test with inputs varying more slowly
    config.numSamples = 40;  // duration of each command in number of samples (2 sec per step)
    for(i = 0; i < 20; i += 1)
    {
        // specify commands
        config.command.forward = rand() % 40 + 11;
        config.command.left    = (rand() % 81) - 40;
        
        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";
        
        run_test(config);
    }
    
    // Go back to the neural position for a few second.
    std::cout<<"INFO: Finished test runs for the long random inputs...\n"
             <<std::endl;
    std::cout<<"INFO: Data collection finished.\n"
             <<std::endl;
    
    config.numSamples = static_cast<int>(sleepTime/SAMPLING_INTERVAL_S);
    config.command.forward = 0;
    config.command.left    = 0;
    run_test(config);
}
