/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     quantum_6000_calibration_maneuvers_with_encoders.cpp
* \author   Jong Jin Park
*
* Data collecting maneuvers for a quantum 6000 wheelchair with imu and without encoders
*/

#include "utils/command_line.h"
#include "utils/timestamp.h"
#include "sensors/microstrain_3dmgx2.h"
#include "robot/quantum_6000.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>


using namespace vulcan;


const std::string JOYSTICK_PORT("joystick-port");
const std::string CONTROLLER_PORT("controller-port");
const std::string IMU("imu-port");
const std::string FILENAME("output-file");


struct test_configuration_t
{
    test_configuration_t(robot::Quantum6000&          quantum,
                         sensors::Microstrain3DMGX2&  imu,
                         int                          samples,
                         robot::joystick_command_t&   command)
    : quantum(quantum)
    , imu(imu)
    , testSamples(samples)
    , command(command)
    {
    }

    robot::Quantum6000&         quantum;
    sensors::Microstrain3DMGX2& imu;
    int                         testSamples;
    robot::joystick_command_t   command;
};

struct test_sample_t
{
    test_sample_t(const robot::joystick_command_t& command, float angular)
    : command(command)
    , angularVelocity(angular)
    {
    }

    robot::joystick_command_t command;
    float                     angularVelocity;
};


void display_help_if_needed(const utils::CommandLine& commandLine);
bool isJoystickCentered(const robot::joystick_command_t& joystick);

void run_maneuvers(test_configuration_t& config, std::ofstream& file);
void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples);
void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file);

/**
* 7/03/2013 quantum_6000_calibration_maneuvers_noEncoders is used to determine the mapping between joystick position
* and the wheel speeds
*
* The code is based on the quantum_6000_parameter_estimator and quantum_6000_calibration_maneuvers.cpp
*
* The maneuvers consists of long sequence of pre-determined step tests, random steps and chirp tests;
*
* The command-line parameters for quantum_6000_calibration_maneuvers are:
*
*   --joystick-port    'port'        CANBus port to which the joystick is connected
*   --controller-port  'port'        CANBus port to which the controller is connected
*   --imu-port         'port'        Port for the IMU
*   --output-file      'filename'    File in which the data should be written
*/

int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    // get ports from command line
    std::string        joystickPort(commandLine.argumentValue(JOYSTICK_PORT));
    std::string        controllerPort(commandLine.argumentValue(CONTROLLER_PORT));
    robot::Quantum6000 quantum(controllerPort, joystickPort);

    // start Quantum thread
    utils::Thread quantumThread;
    quantumThread.attachTask(&quantum);
    quantumThread.start();

    // start IMU
    std::string imuPort(commandLine.argumentValue(IMU));
    sensors::Microstrain3DMGX2 imu(imuPort);
    imu.startIMU();

    sleep(1);

    // initiate joystick
    robot::joystick_command_t command(0, 0, 100);

    std::cout<<"Ready to run. Switch over to robot control. Waiting 5 seconds...\n";
    sleep(5);

    // initialize test configuration, specify save file, and run maneuvers
    int testSamples_dummy = 100;
    test_configuration_t base(quantum, imu, testSamples_dummy, command);

    std::ofstream samplesFile(commandLine.argumentValue(FILENAME));

    run_maneuvers(base, samplesFile);

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool helpNeeded = !commandLine.argumentExists(JOYSTICK_PORT)    ||
                      !commandLine.argumentExists(CONTROLLER_PORT)  ||
                      !commandLine.argumentExists(IMU)              ||
                      !commandLine.argumentExists(FILENAME);

    if(helpNeeded)
    {
        std::cout<<"The command-line parameters for quantum_6000_parameter_estimation are:\n"
                 <<'\n'
                 <<"--joystick-port    'port'        CANBus port to which the joystick is connected\n"
                 <<"--controller-port  'port'        CANBus port to which the controller is connected\n"
                 <<"--imu-port         'port'        Port for the IMU\n"
                 <<"--output-file      'filename'    File in which the data should be written\n"
                 <<std::endl;
        exit(1);
    }
}

bool isJoystickCentered(const robot::joystick_command_t& joystick)
{
    return (joystick.forward == 0) && (joystick.left == 0);
}

void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples)
{
    float                     angular_dummy = 0.0f;
    imu_data_t       data;
    robot::joystick_command_t humanJoystickCommand;

    test_sample_t sample(config.command, angular_dummy);

    std::cout<<"INFO: Starting test: Command:("<<config.command.forward<<','<<config.command.left<<") Samples:"<<config.testSamples<<'\n';

    for(int n = config.testSamples; --n >= 0;)
    {
        // sample sensor data
        config.imu.getIMUData(data);
        sample.angularVelocity = -data.rotationalVelocity[IMU_YAW_INDEX];

        // check current human command, set and sample command data.
        humanJoystickCommand = config.quantum.getJoystickCommand();
        if(!isJoystickCentered(humanJoystickCommand)) // set command if there is no external joystick command.
        {
            config.command.timestamp = utils::system_time_us();
            config.quantum.setJoystickCommand(config.command);
            sample.command = config.command;
        }
        else
        {
            sample.command = humanJoystickCommand; // simply record external joystick command if it is present.
        }

        // save the sample
        samples.push_back(sample);

        // hold for desired microseconds
        usleep(50000);
    }
}


void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file)
{
    for(unsigned int n = 0; n < samples.size(); n++)
    {
      file<<samples[n].command.timestamp<<' '<<samples[n].command.forward<<' '<<samples[n].command.left<<' '<<samples[n].angularVelocity<<'\n';
    }
}


void run_maneuvers(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"This is a test to evaluate the behavior of the wheelchair under changing command inputs."
             <<"\n";

    // initializations
    std::vector<test_sample_t> velocitySamples;
    int   i = 0;
    int   j = 0;
    float sleepTime = 3;

    // start at neutral position and hold for 1 second
    config.command.timestamp = utils::system_time_us();
    config.command.forward = 0;
    config.command.left    = 0;
    config.quantum.setJoystickCommand(config.command);

    sleep(1);

    ////////////////////////////////////////////////////////////////////////////
    // run step tests (alternates between larger and smaller step inputs)
    config.testSamples = 70;   // duration of each commands in number of samples (3.5 sec per step)
    for(i = 0; i < 50; i += 7)
    {
        for(j = -49; j < 50; j +=7)
        {
            // larger step commands
            config.command.forward = i;
            config.command.left    = j;

            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

            run_test(config, velocitySamples);

            // smaller step commands
            config.command.forward = i*2/7;
            if(j == 0)
            {
                config.command.left = 0;
            }
            else
            {
                config.command.left = copysign(2*(7-abs(j)/7),j);
            }

            std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

            run_test(config, velocitySamples);
        }

    // Go back to the neural position for a few second.
    config.command.timestamp = utils::system_time_us();
    config.command.forward = 0;
    config.command.left    = 0;
    config.quantum.setJoystickCommand(config.command);

    std::cout<<"INFO: Finished " << (i+7)/7 << "/8 of the sweeps for step inputs...\n"
             <<std::endl;

    sleep(sleepTime);

    }

    ////////////////////////////////////////////////////////////////////////////
    // run tests with sinusoidal inputs
    config.testSamples = 1;     // duration of each commands in number of samples
    for(i = 0; i < 200; i += 1) // 10 sec each for a sinusoid
    {
        // specify commands
        config.command.forward = 20.0*sin(2.0*3.14/100.0*i + 3.14) + 30.0; //  10 < y < 50
        config.command.left    = 40.0*sin(2.0*3.14/200.0*i + 3.14/4.0);    // -40 < x < 40

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 20.0*sin(2.0*3.14/100.0*i + 3.14/4.0) + 30.0; //  10 < y < 50
        config.command.left    = 40.0*sin(2.0*3.14/200.0*i + 3.14/4.0);        // -40 < x < 40

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 10.0*sin(2.0*3.14/100.0*i + 2*3.14/4.0) + 20.0; //  10 < x < 30
        config.command.left    = 30.0*sin(2.0*3.14/200.0*i + 3.14/4.0);          // -30 < x < 30

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }
    for(i = 0; i < 200; i += 1)
    {
        // specify commands
        config.command.forward = 10.0*sin(2.0*3.14/100.0*i + 3*3.14/4.0) + 20.0; //  10 < y < 30
        config.command.left    = 30.0*sin(2.0*3.14/200.0*i + 3.14/4.0);          // -30 < x < 30

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }

    // Go back to the neural position for a few second.
    config.command.timestamp = utils::system_time_us();
    config.command.forward = 0;
    config.command.left    = 0;
    config.quantum.setJoystickCommand(config.command);

    std::cout<<"INFO: Finished test runs for the sinusoidal inputs...\n"
             <<"INFO: Continuing to radom inputs...\n"
             <<std::endl;

    sleep(sleepTime);

    ///////////////////////////////////////////////////////////////////////////
    // run random test with quickly varying inputs
    config.testSamples = 10;    // duration of each commands in number of samples (0.5 sec per step)
    for(i = 0; i < 40; i += 1)
    {
        // specify commands
        config.command.left    = (rand() % 81) - 40; // -40 < x < 40
        config.command.forward = rand() % 40 + 11;   //  11 < y < 50

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }

    // Go back to the neural position for a few second.
    config.command.timestamp = utils::system_time_us();
    config.command.forward = 0;
    config.command.left    = 0;
    config.quantum.setJoystickCommand(config.command);

    std::cout<<"INFO: Finished test runs for the short random inputs...\n"
             <<"INFO: Continuing to long random inputs...\n"
             <<std::endl;
    sleep(sleepTime);

    ///////////////////////////////////////////////////////////////////////////
    // run random test with inputs varying more slowly
    config.testSamples = 40;  // duration of each command in number of samples (2 sec per step)
    for(i = 0; i < 20; i += 1)
    {
        // specify commands
        config.command.left    = (rand() % 81) - 40;
        config.command.forward = rand() % 40 + 11;

        std::cout<<"Command set: ("<<config.command.forward<<','<<config.command.left<<")\n";

        run_test(config, velocitySamples);
    }

    // Go back to the neural position for a few second.
    config.command.timestamp = utils::system_time_us();
    config.command.forward = 0;
    config.command.left    = 0;
    config.quantum.setJoystickCommand(config.command);

    std::cout<<"INFO: Finished test runs for the long random inputs...\n"
             <<std::endl;
    std::cout<<"INFO: Data collection finished.\n"
             <<std::endl;
    sleep(sleepTime);


    // save collected samples to a file
    save_test_samples(velocitySamples, file);

}
