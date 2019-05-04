/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cstdlib>

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sensors/imu.h>
#include <core/imu_data.h>

#include <sensors/microstrain_3dmgx2.h>
#include <sensors/microstrain_3dmgx3.h>
#include <system/module_communicator.h>

#include <utils/command_line.h>
#include <utils/timestamp.h>


using vulcan::utils::CommandLine;
using vulcan::utils::system_time_us;
using vulcan::sensors::IMU;
using vulcan::imu_data_t;
using vulcan::system::ModuleCommunicator;


// Definition of the command-line argument names
const std::string HELP_LONG("help");
const std::string HELP_SHORT("h");
const std::string IMU_MODEL("imu-model");
const std::string IMU_PORT("port");
const std::string CHANNEL("channel");
const std::string BIAS_SAMPLES("bias-samples");

// Definition of driver names
const std::string GX2("3DM-GX2");
const std::string GX3("3DM-GX3");

// Default values for command-line arguments
const std::string DEFAULT_IMU_MODEL(GX2);
const std::string DEFAULT_IMU_PORT("/dev/ttyUSB0");
const std::string DEFAULT_CHANNEL("SENSOR_IMU");
const std::string DEFAULT_BIAS_SAMPLES("500");


// Helpers for processing the command line
bool display_help_message_if_needed(const CommandLine& commandLine);

// Helpers for loading the correct drivers
boost::shared_ptr<IMU> load_imu_driver(const CommandLine& commandLine);
boost::shared_ptr<IMU> load_microstrain_3dmgx2_driver(const CommandLine& commandLine);
boost::shared_ptr<IMU> load_microstrain_3dmgx3_driver(const CommandLine& commandLine);

void estimate_gravity(boost::shared_ptr<IMU> imu, const CommandLine& commandLine);

// Helpers for loading the correct communication protocol
boost::shared_ptr<ModuleCommunicator> load_imu_data_transmitter(const CommandLine& commandLine);

// Helpers for doing the actual data transmission
void produce_imu_data(boost::shared_ptr<IMU> imu, boost::shared_ptr<ModuleCommunicator> transmitter, const std::string& channel);

/**
* imu_data_producer is a program that reads data from a three-axis IMU and then
* transmits it to all modules that need the data. The imu_data_producer supports
* any three-axis IMU driver that implement the IMU interface.
*
* Command-line arguments are used to pick the driver and communication protocol to use.
*
* The command-line arguments for imu_data_producer are as follows:
*
*   -h, --help                  Print this message
*   --port 'IMU port'           Port the device is connected to (supplied to driver) : Default = /dev/ttyUSB0
*   --imu-model 'IMU type'      IMU driver to use : Default = 3DM-GX2
*           Currently supported IMUs:
*               3DM-GX2
*               3DM-GX3
*   --channel 'name'            Name of the output channel for the data : Default = SENSOR_IMU
*   --bias-samples 'number'     Number of samples to take for estimating the acceleration vector : Default = 500
*/
int main(int argc, char** argv)
{
    CommandLine commandLine(argc, argv);
    // If help requested, show it and then exit immediately
    if(display_help_message_if_needed(commandLine))
    {
        return -1;
    }

    boost::shared_ptr<IMU> imu = load_imu_driver(commandLine);

    estimate_gravity(imu, commandLine);

    boost::shared_ptr<ModuleCommunicator> transmitter = load_imu_data_transmitter(commandLine);

    std::string channel = commandLine.argumentExists(CHANNEL) ? commandLine.argumentValue(CHANNEL) : DEFAULT_CHANNEL;

    produce_imu_data(imu, transmitter, channel);

    return 0;
}


// Helpers for processing the command line
bool display_help_message_if_needed(const CommandLine& commandLine)
{
    // Help is needed if explicitly requested or if the IMU driver and communication protocol
    // are not specified
    bool needHelp = commandLine.argumentExists(HELP_SHORT)   ||
                    commandLine.argumentExists(HELP_LONG);

    if(needHelp)
    {
        std::cout<<"imu_data_producer: \n"
                 <<"Command-line arguments are used to pick the driver and communication protocol to use.\n"
                 <<'\n'
                 <<"The command-line arguments for imu_data_producer are as follows:\n"
                 <<'\n'
                 <<"    -h, --help                  Print this message\n"
                 <<"    --port 'IMU port'           Port the device is connected to (supplied to driver) : Default = "<<DEFAULT_IMU_PORT<<'\n'
                 <<"    --imu-model 'IMU type'      IMU driver to use : Default = "<<DEFAULT_IMU_MODEL<<'\n'
                 <<"            Currently supported IMUs:\n"
                 <<"                3DM-GX2\n"
                 <<"                3DM-GX3\n"
                 <<"    --channel 'name'            Name of the output channel for the data : Default = "<<DEFAULT_CHANNEL<<'\n'
                 <<"    --bias-samples 'number'     Number of samples to take for estimating the acceleration vector : Default = "<<DEFAULT_BIAS_SAMPLES<<'\n'
                 <<std::endl;
    }

    return needHelp;
}


// Helpers for loading the correct drivers
boost::shared_ptr<IMU> load_imu_driver(const CommandLine& commandLine)
{
    std::string imuModel = commandLine.argumentExists(IMU_MODEL) ? commandLine.argumentValue(IMU_MODEL) : DEFAULT_IMU_MODEL;

    assert(!imuModel.empty());

    boost::shared_ptr<IMU> imu;

    if(imuModel == GX2)
    {
        imu = load_microstrain_3dmgx2_driver(commandLine);
    }
    else if(imuModel == GX3)
    {
        imu = load_microstrain_3dmgx3_driver(commandLine);
    }

    // Enforce that a driver was loaded. If nothing loaded, no point in continuing
    assert(imu.get());

    imu->startIMU();

    return imu;
}


boost::shared_ptr<IMU> load_microstrain_3dmgx2_driver(const CommandLine& commandLine)
{
    std::string imuPort = commandLine.argumentExists(IMU_PORT) ? commandLine.argumentValue(IMU_PORT) : DEFAULT_IMU_PORT;

    assert(!imuPort.empty());

    const int64_t gx2TicksPerSecond = 19660800;

    return boost::shared_ptr<IMU>(new vulcan::sensors::Microstrain3DMGX2(imuPort,
                                                                         gx2TicksPerSecond));
}


boost::shared_ptr<IMU> load_microstrain_3dmgx3_driver(const CommandLine& commandLine)
{
    std::string imuPort = commandLine.argumentExists(IMU_PORT) ? commandLine.argumentValue(IMU_PORT) : DEFAULT_IMU_PORT;

    assert(!imuPort.empty());

    const int64_t gx3TicksPerSecond = 16000000;

    return boost::shared_ptr<IMU>(new vulcan::sensors::Microstrain3DMGX2(imuPort, gx3TicksPerSecond));
}


void estimate_gravity(boost::shared_ptr<IMU> imu, const CommandLine& commandLine)
{
    std::string biasString = commandLine.argumentExists(BIAS_SAMPLES) ? commandLine.argumentValue(BIAS_SAMPLES) : DEFAULT_BIAS_SAMPLES;

    uint16_t biasSamples = atoi(biasString.c_str());

    std::cout<<"imu_data_producer: Estimating IMU gravity vector using "<<biasSamples<<" samples\n";

    imu->estimateGravityMagnitude(biasSamples);
}


// Helpers for loading the correct communication protocol
boost::shared_ptr<ModuleCommunicator> load_imu_data_transmitter(const CommandLine& commandLine)
{
    return boost::shared_ptr<ModuleCommunicator>(new ModuleCommunicator());
}


// Helpers for doing the actual data transmission
void produce_imu_data(boost::shared_ptr<IMU> imu, boost::shared_ptr<ModuleCommunicator> transmitter, const std::string& channel)
{
    int64_t startTime   = 0;
    int64_t endTime     = 0;
    int64_t deltaTime   = 0;
    int     numReadings = 0;

    imu_data_t latestIMUData;
    int sequenceNumber = 0;

    while(true)
    {
        startTime = system_time_us();

        if(imu->getIMUData(latestIMUData))
        {
            endTime    = system_time_us();
            deltaTime += (endTime > startTime) ? endTime-startTime : startTime-endTime;
            ++numReadings;

            if(deltaTime > vulcan::utils::sec_to_usec(1))
            {
                std::cout << "IMU update rate: " << (static_cast<float>(numReadings) * vulcan::utils::usec_to_sec(deltaTime))
                    << " Hz" << std::endl;

                numReadings = 0;
                deltaTime   = 0;
            }
            
            latestIMUData.sequenceNumber = sequenceNumber++;

            // The IMU reference frame is backwards from the robot reference frame
            latestIMUData.rotationalVelocity[vulcan::IMU_YAW_INDEX] *= -1;

            transmitter->sendMessage(latestIMUData, channel);
        }
    }
}
