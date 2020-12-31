/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include <fstream>
#include <iostream>
#include "utils/config_file.h"
#include "utils/command_line.h"
#include "utils/timestamp.h"
#include "utils/repeated_task.h"
#include "system/module_communicator.h"
#include "sensors/odometry_estimator.h"
#include "core/odometry.h"
#include <unistd.h>

using namespace vulcan;

// Definition of the command-line argument names
const std::string HELP_LONG("help");
const std::string HELP_SHORT("h");
const std::string TYPE("type");
const std::string CHANNEL("channel");
const std::string LOG_FILE("log-file");
const std::string CONFIG_FILE("config-file");
const std::string UPDATE_RATE("update-rate");

const std::string DEFAULT_TYPE("wheel_encoders");
const std::string DEFAULT_CHANNEL("SENSOR_ODOMETRY");
const std::string DEFAULT_RATE("50");


void display_help_if_needed(const utils::CommandLine& commandLine);
void produce_odometry(std::unique_ptr<sensors::OdometryEstimator>& estimator, int updateRate, const std::string& channel, const std::string& logName);
void log_odometry(const odometry_t& odometry, std::ofstream& log);


/**
* odometry_producer uses wheel encoders to calculate dead reckoning odometry for the robot.
* Odometry data is sent out at 50Hz by default, but can be changed on the command-line.
*
* The command-line arguments for odometry_producer are:
*
*   -h/--help                   Display this help message
*   --type 'estimator'          Type of OdometryEstimator to create (default = wheel_encoders)
*   --channel 'name'            Channel on which to publish the odometry data (optional, default = SENSOR_ODOMETRY)
*   --config-file 'filename'    Name of the configuration file for the encoders driver
*   --update-rate 'hz'          Hertz at which odometry data should be transmitted (optional, default = 50)
*   --log-file    'filename'    Filename of log in which to store the odometry data (optional)
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    utils::ConfigFile config(commandLine.argumentValue(CONFIG_FILE));

    std::unique_ptr<sensors::OdometryEstimator> estimator = sensors::create_odometry_estimator(commandLine.argumentValue(TYPE, DEFAULT_TYPE),
                                                                                               config);

    int updateRate = atoi(commandLine.argumentValue(UPDATE_RATE, DEFAULT_RATE).c_str());

    produce_odometry(estimator, updateRate, commandLine.argumentValue(CHANNEL, DEFAULT_CHANNEL), commandLine.argumentValue(LOG_FILE));

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool needHelp = commandLine.argumentExists(HELP_LONG)  ||
                    commandLine.argumentExists(HELP_SHORT) ||
                    !commandLine.argumentExists(CONFIG_FILE);

    if(needHelp)
    {
        std::cout<<"The command-line arguments for odometry_producer are:\n"
                 <<'\n'
                 <<"   -h/--help                   Display this help message\n"
                 <<"   --type 'estimator'          Type of OdometryEstimator to create (default = wheel_encoders)\n"
                 <<"   --channel 'name'            Channel on which to publish the odometry data (default = SENSOR_ODOMETRY)\n"
                 <<"   --config-file 'filename'    Name of the configuration file for the encoders driver\n"
                 <<"   --update-rate 'hz'          Hertz at which odometry data should be transmitted (optional, default = 50)\n"
                 <<"   --log-file    'filename'    Filename of log in which to store the odometry data (optional)"
                 <<std::endl;

        exit(0);
    }
}


void produce_odometry(std::unique_ptr<sensors::OdometryEstimator>& estimator, int updateRate, const std::string& channel, const std::string& logName)
{
    system::ModuleCommunicator    communicator;
    system::ModuleCommunicator transmitter;
    odometry_t            odometry;

    std::ofstream log;

    if(!logName.empty())
    {
        log.open(logName);
    }

    int64_t sleepTime = utils::sec_to_usec(1.0 / updateRate);

    int64_t startTime   = utils::system_time_us();
    int64_t deltaTime   = 0;
    int     numReadings = 0;
    
    estimator->initialize(communicator);
    
    auto communicatorFunc = [&communicator](bool stopping) -> bool { communicator.processIncoming(); return false; };
    utils::RepeatedTask producerTask(communicatorFunc);

    while(true)
    {
        odometry = estimator->update();

        estimator->send(transmitter);

        deltaTime = utils::system_time_us() - startTime;
        ++numReadings;

        if(deltaTime > utils::sec_to_usec(1))
        {
            std::cout << "Odometry update rate: " << (static_cast<float>(numReadings)*vulcan::utils::usec_to_sec(deltaTime))
                << " Hz" << std::endl;

            numReadings = 0;
            startTime   = utils::system_time_us();
        }

        if(log.is_open())
        {
            log_odometry(odometry, log);
        }

        usleep(sleepTime);
    }
}


void log_odometry(const odometry_t& odometry, std::ofstream& log)
{

}
