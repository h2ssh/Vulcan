/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_joystick_with_encoders.cpp
* \author   Ron Gaynier
*
* Data collecting maneuvers for a quantum 6000 wheelchair with imu and encoders
*/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <utils/command_line.h>
#include <utils/timestamp.h>
#include <utils/config_file.h>
#include <sensors/microstrain_3dmgx2.h>
#include <sensors/wheel_encoders.h>
#include <robot/quantum_6000.h>


using namespace vulcan;


const std::string JOYSTICK_PORT("joystick-port");
const std::string CONTROLLER_PORT("controller-port");
const std::string IMU("imu-port");
const std::string FILENAME("output-file");
const std::string CONFIG_FILE("config-file");

std::unique_ptr<sensors::WheelEncoders> encoders;

struct test_configuration_t
{
    test_configuration_t(robot::Quantum6000&         quantum,
                         sensors::Microstrain3DMGX2& imu,
                         int                         samples,
                         robot::joystick_command_t&  command)
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
    test_sample_t(const robot::joystick_command_t& command, float angular, const encoder_data_t& encoderData)
    : command(command)
    , angularVelocity(angular)
    , encoderData(encoderData)
    {
    }

    robot::joystick_command_t   command;
    float                       angularVelocity;
    encoder_data_t     encoderData;
};


void display_help_if_needed(const utils::CommandLine& commandLine);

void run_maneuvers(test_configuration_t& config, std::ofstream& file);

void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples);
void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file);

/**
* 8/05/2013 quantum_6000_calibration_maneuvers_withEncoders is used to determine the mapping between joystick position
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
*   --config-file      'filename'    Name of the configuration file for the encoders driver
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
    
    // start encoders
    utils::ConfigFile config(commandLine.argumentValue(CONFIG_FILE));
    sensors::wheel_encoders_params_t params = sensors::load_wheel_encoders_params(config);
    encoders = sensors::create_wheel_encoders(params.encoderType, params);

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
                      !commandLine.argumentExists(CONFIG_FILE)      ||
                      !commandLine.argumentExists(FILENAME);

    if(helpNeeded)
    {
        std::cout<<"The command-line parameters for quantum_6000_parameter_estimation are:\n"
                 <<'\n'
                 <<"--joystick-port    'port'        CANBus port to which the joystick is connected\n"
                 <<"--controller-port  'port'        CANBus port to which the controller is connected\n"
                 <<"--imu-port         'port'        Port for the IMU\n"
                 <<"--config-file      'filename'    Name of the configuration file for the encoders driver\n"
                 <<"--output-file      'filename'    File in which the data should be written\n"
                 <<std::endl;
        exit(1);
    }
}


void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples)
{
    float                   angular_dummy = 0.0f;
    imu_data_t     data;
    encoder_data_t encoderData;
   
    test_sample_t sample(config.command, angular_dummy, encoderData);

    std::cout<<"INFO: Starting data collection"<<"\n";

    for(int n = 2500; --n >= 0;)
    {
        // sample sensor data
        config.imu.getIMUData(data);
        sample.angularVelocity = -data.rotationalVelocity[IMU_YAW_INDEX];
        encoders->update();
        sample.encoderData = encoders->getEncoders(); 
        
        // set command and sample command data
        // config.command.timestamp = utils::system_time_us();
        sample.command = config.quantum.getJoystickCommand();

        // save the sample
        samples.push_back(sample);

        // hold for desired microseconds (50000 = 20Hz)
        usleep(50000);

    }
}


void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file)
{
    for(unsigned int n = 0; n < samples.size(); n++)
    {
      file<<samples[n].command.timestamp              <<' '
          <<samples[n].command.forward                <<' '<<samples[n].command.left                    <<' '
          <<samples[n].encoderData.id                 <<' '<<samples[n].encoderData.timestamp           <<' '
          <<samples[n].encoderData.leftIndexPulseTotal<<' '<<samples[n].encoderData.rightIndexPulseTotal<<' '
          <<samples[n].encoderData.leftTicksTotal     <<' '<<samples[n].encoderData.rightTicksTotal     <<' '
          <<samples[n].angularVelocity<<'\n';
    }
}


void run_maneuvers(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"This simply logs the Joystick command and the data from the Encoders."
             <<"\n";

    // initializations
    std::vector<test_sample_t> velocitySamples;
    
    // encoders->resetEncoderData();

    // start at neutral position and hold for 1 second
    // config.command.timestamp = utils::system_time_us();
    // config.command.forward = 0;
    // config.command.left    = 0;
    // config.quantum.setJoystickCommand(config.command);
    config.command = config.quantum.getJoystickCommand();

    sleep(1);
   
    run_test(config, velocitySamples);
   
    // save collected samples to a file
    save_test_samples(velocitySamples, file);

    std::cout<<"INFO: Ended data collection"<<"\n";

}
