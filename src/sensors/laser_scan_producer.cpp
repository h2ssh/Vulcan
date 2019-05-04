/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <string>
#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include <core/pose.h>
#include <math/geometry/rectangle.h>
#include <sensors/laser_rangefinder.h>
#include <core/laser_scan.h>
#include <laser/laser_io.h>

#include <sensors/hokuyo_urg_laser.h>
#include <system/module_communicator.h>

#include <utils/command_line.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>
#include <utils/timestamp.h>

using namespace vulcan;


// Definition of the command-line argument names
const std::string HELP_LONG("help");
const std::string HELP_SHORT("h");
const std::string LASER_MODEL("laser-model");
const std::string WITH_INTENSITY("with-intensity");
const std::string LASER_PORT("port");
const std::string CHANNEL("channel");
const std::string ID("id");
const std::string LOG_FILE("log-file");
const std::string CONFIG_FILE("config-file");

// Definition of driver names
const std::string UTM_30LX("UTM-30LX");
const std::string URG_04LX("URG-04LX");

// Default command-line argument values
const std::string DEFAULT_LASER_MODEL(UTM_30LX);
const std::string DEFAULT_PORT("/dev/ttyACM0");
const std::string DEFAULT_CHANNEL("SENSOR_FRONT_LASER");
const std::string DEFAULT_ID("0");

// ConfigFile heading and key definitions
const std::string PRODUCER_HEADING("LaserScanProducerParameters");
const std::string OFFSET_KEY      ("laser_offset");
const std::string BOUNDARY_KEY    ("robot_boundary");
const std::string INTENSITY_KEY   ("with_intensity");
const std::string FIRST_VALID_KEY ("first_valid_index");
const std::string LAST_VALID_KEY  ("last_valid_index");
const std::string CHANNEL_KEY     ("channel");
const std::string PORT_KEY        ("port");
const std::string ID_KEY          ("id");
const std::string MODEL_KEY       ("laser_model");


struct laser_params_t
{
    pose_6dof_t     offset;
    math::Rectangle<float> robotBoundary;
    bool                   withIntensity;
    std::size_t            firstValidIndex;
    std::size_t            lastValidIndex;
    std::string            channel;
    std::string            port;
    std::string            model;
    int                    id;
};


// Helpers for processing the command line
bool display_help_message_if_needed(const utils::CommandLine& commandLine);

laser_params_t load_laser_params(const utils::CommandLine& commandLine);

// Helpers for loading the correct drivers
boost::shared_ptr<sensors::LaserRangefinder> load_rangefinder_driver(const laser_params_t& params);
boost::shared_ptr<sensors::LaserRangefinder> load_hokuyo_urg_laser_driver(const laser_params_t& params);

// Helpers for loading the correct communication protocol
boost::shared_ptr<system::ModuleCommunicator> load_scan_transmitter(const laser_params_t& params);

std::string data_description(const utils::CommandLine& commandLine);
std::string log_filename    (const utils::CommandLine& commandLine);

// Helpers for doing the actual data transmission
void produce_laser_scans(boost::shared_ptr<sensors::LaserRangefinder>      rangefinder,
                         boost::shared_ptr<system::ModuleCommunicator> transmitter,
                         const laser_params_t&                             params,
                         const std::string&                                logFilename);

void invalidate_scan_points(polar_laser_scan_t& scan, size_t firstIndex, size_t lastIndex);
void invalidate_scan_points_hitting_robot(polar_laser_scan_t&           polar,
                                          const cartesian_laser_scan_t& cartesian,
                                          const math::Rectangle<float>&        boundary);

/**
* laser_scan_producer is a program that reads data from a laser rangefinder and then
* transmits it to all modules that need the data. The laser_scan_producer supports
* any laser rangefinder drivers that implement the LaserRangefinder interface.
* Transmission of laser scans uses the LaserScanTransmitter interface, so any implementation
* is supported.
*
* Command-line arguments are used to  pick the driver and communication protocol to use.
*
* The command-line arguments for laser_scan_producer are as follows:
*
*   -h, --help                  Print this message
*   --port 'laser port'         Port the device is connected to (supplied to driver) : Default = /dev/ttyACM0
*   --laser-model 'laser type'  Rangefinder driver to use : Default = UTM-30LX
*           Currently supported lasers:
*               UTM-30LX
*               URG-04LX
*   --config-file 'filename'    Name of the config file with settings about the laser
*   --id          'id'          Unique id for the laser
*   --with-intensity            If present, then capture intensity data along with the range data (optional)
*   --channel 'name'            Name of the channel for the data : Default = SENSOR_FRONT_LASER
*   --log-file 'filename'       Name of file in which to save the scan data being produced. If the option isn't set, data isn't logged (optional)
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    // If help requested, show it and then exit immediately
    if(display_help_message_if_needed(commandLine))
    {
        return -1;
    }

    laser_params_t params = load_laser_params(commandLine);

    boost::shared_ptr<sensors::LaserRangefinder>       rangefinder = load_rangefinder_driver(params);
    boost::shared_ptr<system::ModuleCommunicator>  transmitter = load_scan_transmitter(params);
    std::string                                        logFilename = log_filename(commandLine);

    produce_laser_scans(rangefinder, transmitter, params, logFilename);

    return 0;
}


// Helpers for processing the command line
bool display_help_message_if_needed(const utils::CommandLine& commandLine)
{
    // Help is needed if explicitly requested or if the laser driver and communication protocol
    // are not specified
    bool needHelp = commandLine.argumentExists(HELP_SHORT)   ||
                    commandLine.argumentExists(HELP_LONG)    ||
                    !commandLine.argumentExists(CONFIG_FILE);

    if(needHelp)
    {
        std::cout<<"laser_scan_producer: \n"
                 <<"Command-line arguments are used to  pick the driver and communication protocol to use.\n"
                 <<'\n'
                 <<"The command-line arguments for laser_scan_producer are as follows:\n"
                 <<'\n'
                 <<"    -h, --help                  Print this message\n"
                 <<"    --port 'laser port'         Port the device is connected to (supplied to driver) : Default = "<<DEFAULT_PORT<<'\n'
                 <<"    --laser-model 'laser type'  Rangefinder driver to use : Default = "<<DEFAULT_LASER_MODEL<<'\n'
                 <<"            Currently supported lasers:\n"
                 <<"                UTM-30LX\n"
                 <<"                URG-04LX\n"
                 <<"    --config-file 'filename'    Name of the config file with settings about the laser\n"
                 <<"    --with-intensity            If present, then capture intensity data along with the range data (optional)\n"
                 <<"    --channel 'name'            Name of the channel for the data : Default = "<<DEFAULT_CHANNEL<<'\n'
                 <<"    --id      'id'              Unique id for the laser producing the data : Default = "<<DEFAULT_ID<<'\n'
                 <<"    --log-file 'filename'       Name of file in which to save the scan data being produced. If the option isn't set, data isn't logged (optional)\n"
                 <<std::endl;
    }

    return needHelp;
}


laser_params_t load_laser_params(const utils::CommandLine& commandLine)
{
    utils::ConfigFile config(commandLine.argumentValue(CONFIG_FILE));

    std::vector<std::pair<std::string, std::string>> requiredConfigValues;

    requiredConfigValues.push_back(std::make_pair(PRODUCER_HEADING, OFFSET_KEY));
    requiredConfigValues.push_back(std::make_pair(PRODUCER_HEADING, BOUNDARY_KEY));
    requiredConfigValues.push_back(std::make_pair(PRODUCER_HEADING, FIRST_VALID_KEY));
    requiredConfigValues.push_back(std::make_pair(PRODUCER_HEADING, LAST_VALID_KEY));

    if(config.validate(requiredConfigValues) > 0)
    {
        std::cerr<<"ERROR: Missing required config file values."<<std::endl;
        exit(-1);
    }

    laser_params_t params;

    params.offset        = utils::create_pose_6dof_from_string(config.getValueAsString(PRODUCER_HEADING, OFFSET_KEY));
    params.robotBoundary = utils::create_rectangle_from_string(config.getValueAsString(PRODUCER_HEADING, BOUNDARY_KEY));
    params.firstValidIndex = config.getValueAsUInt32(PRODUCER_HEADING, FIRST_VALID_KEY);
    params.lastValidIndex  = config.getValueAsUInt32(PRODUCER_HEADING, LAST_VALID_KEY);

    assert(params.firstValidIndex <= params.lastValidIndex);
    assert(params.firstValidIndex >= 0 && params.lastValidIndex >= 0);

    if(!commandLine.argumentExists(CHANNEL) && config.hasValue(PRODUCER_HEADING, CHANNEL_KEY))
    {
        params.channel = config.getValueAsString(PRODUCER_HEADING, CHANNEL_KEY);
    }
    else
    {
        params.channel = commandLine.argumentValue(CHANNEL, DEFAULT_CHANNEL);
    }

    if(!commandLine.argumentExists(LASER_PORT) && config.hasValue(PRODUCER_HEADING, PORT_KEY))
    {
        params.port = config.getValueAsString(PRODUCER_HEADING, PORT_KEY);
    }
    else
    {
        params.port = commandLine.argumentValue(LASER_PORT, DEFAULT_PORT);
    }

    if(!commandLine.argumentExists(ID) && config.hasValue(PRODUCER_HEADING, ID_KEY))
    {
        params.id = config.getValueAsInt32(PRODUCER_HEADING, ID_KEY);
    }
    else
    {
        params.id = atoi(commandLine.argumentValue(ID, DEFAULT_ID).c_str());
    }

    if(!commandLine.argumentExists(LASER_MODEL) && config.hasValue(PRODUCER_HEADING, MODEL_KEY))
    {
        params.model = config.getValueAsString(PRODUCER_HEADING, MODEL_KEY);
    }
    else
    {
        params.model = commandLine.argumentValue(LASER_MODEL, DEFAULT_LASER_MODEL);
    }

    if(!commandLine.argumentExists(WITH_INTENSITY) && config.hasValue(PRODUCER_HEADING, INTENSITY_KEY))
    {
        params.withIntensity = config.getValueAsBool(PRODUCER_HEADING, INTENSITY_KEY);
    }
    else
    {
        params.withIntensity = commandLine.argumentExists(WITH_INTENSITY);
    }

    return params;
}


// Helpers for loading the correct drivers
boost::shared_ptr<sensors::LaserRangefinder> load_rangefinder_driver(const laser_params_t& params)
{
    assert(!params.model.empty());

    boost::shared_ptr<sensors::LaserRangefinder> rangefinder;

    if((params.model == URG_04LX) || (params.model == UTM_30LX))
    {
        rangefinder = load_hokuyo_urg_laser_driver(params);
    }

    // Enforce that a driver was loaded. If nothing loaded, no point in continuing
    assert(rangefinder.get());

    return rangefinder;
}


boost::shared_ptr<sensors::LaserRangefinder> load_hokuyo_urg_laser_driver(const laser_params_t& params)
{
    assert(!params.port.empty());

    return boost::shared_ptr<sensors::LaserRangefinder>(new sensors::HokuyoURGLaser(params.port, params.id, params.withIntensity));
}


// Helpers for loading the correct communication protocol
boost::shared_ptr<system::ModuleCommunicator> load_scan_transmitter(const laser_params_t& params)
{
    return boost::shared_ptr<system::ModuleCommunicator>(new system::ModuleCommunicator());
}


std::string log_filename(const utils::CommandLine& commandLine)
{
    return commandLine.argumentValue(LOG_FILE);
}


// Helpers for doing the actual data transmission
void produce_laser_scans(boost::shared_ptr<sensors::LaserRangefinder>      rangefinder,
                         boost::shared_ptr<system::ModuleCommunicator> transmitter,
                         const laser_params_t&                             params,
                         const std::string&                                logFilename)
{
    int64_t startTime   = 0;
    int64_t endTime     = 0;
    int64_t deltaTime   = 0;
    int     numReadings = 0;

    std::ofstream logFile;

    if(logFilename.length() > 0)
    {
        logFile.open(logFilename.c_str());
    }

    rangefinder->calculateLatency();

    polar_laser_scan_t     polarScan;
    cartesian_laser_scan_t cartesianScan;

    polarScan.offset.x     = params.offset.x;
    polarScan.offset.y     = params.offset.y;
    polarScan.offset.theta = params.offset.theta;
    polarScan.scanId = 0;

    while(true)
    {
        startTime = utils::system_time_us();

        if(rangefinder->getLaserScan(polarScan))
        {
            endTime    = utils::system_time_us();
            deltaTime += (endTime > startTime) ? endTime-startTime : startTime-endTime;
            ++numReadings;

            if(deltaTime > vulcan::utils::sec_to_usec(1))
            {
                std::cout << "Laser update rate: " << (static_cast<float>(numReadings)*vulcan::utils::usec_to_sec(deltaTime))
                    << " Hz" << std::endl;

                numReadings = 0;
                deltaTime   = 0;
            }

            polar_scan_to_cartesian_scan_in_robot_frame(polarScan, cartesianScan, false);
            invalidate_scan_points(polarScan, params.firstValidIndex, params.lastValidIndex);
//             invalidate_scan_points_hitting_robot(polarScan, cartesianScan, params.robotBoundary);

            transmitter->sendMessage(polarScan, params.channel);

            if(logFile.is_open())
            {
                laser::save_laser_scan_to_file(polarScan, logFile);
            }

            ++polarScan.scanId;
        }
    }
}


void invalidate_scan_points(polar_laser_scan_t& scan, size_t firstIndex, size_t lastIndex)
{
    for(uint16_t n = 0; n < scan.numRanges; ++n)
    {
        // Only invalidate the point if it is valid to begin with otherwise bad points will be marked okay
        if((n < firstIndex || n > lastIndex) && (scan.ranges[n] > 0))
        {
            scan.ranges[n] *= -1;
        }
    }
}


void invalidate_scan_points_hitting_robot(polar_laser_scan_t&           polar,
                                          const cartesian_laser_scan_t& cartesian,
                                          const math::Rectangle<float>&        boundary)
{
    // Invalid scan points are those that fall within the bounding rectangle of the robot. The visibility of the lasers
    // overlaps the actual robot. Thus, those points hitting the robot should be ignored. Ignoring laser points means
    // their range is set to a negative number.
    for(uint16_t n = 0; n < cartesian.numPoints; ++n)
    {
        if(boundary.contains(cartesian.scanPoints[n]) || n > 1040)
        {
            polar.ranges[n] *= -1;
        }
    }
}
