/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sensor_log.cpp
* \author   Collin Johnson
*
* Definition of SensorLog.
*/

#include "sensors/sensor_log.h"
#include "system/module_communicator.h"
#include <algorithm>
#include <sstream>

namespace vulcan
{
namespace sensors
{

// A utility class to handle the interaction with the ModuleCommunicator.
class LCMSensorCallbacks
{
public:

    LCMSensorCallbacks(SensorLog& log)
    : log_(log)
    {
    }

    void handleData(const polar_laser_scan_t& scan, const std::string& channel)
    {
        if(scan.laserId == kFrontLaserId)
        {
            log_.addFrontLaser(scan);
        }
        else if(scan.laserId == kBackLaserId)
        {
            log_.addBackLaser(scan);
        }
    }

    void handleData(const imu_data_t& imu, const std::string& channel)
    {
        log_.addImu(imu);
    }

    void handleData(const odometry_t& odom, const std::string& channel)
    {
        log_.addOdometry(odom);
    }

    void handleData(const encoder_data_t& encoders, const std::string& channel)
    {
        log_.addEncoders(encoders);
    }

    void handleData(const robot::commanded_joystick_t& joystick, const std::string& channel)
    {
        log_.addCommandedJoystick(joystick);
    }

    void handleData(const robot::commanded_velocity_t& command, const std::string& channel)
    {
        log_.addCommandedVelocity(command);
    }

private:

    SensorLog& log_;        // Log to add the data to
};


// Utility function for sorting the sensor data by timestamp
template <class Timestamped>
bool sort_by_time(const Timestamped& lhs, const Timestamped& rhs) { return lhs.timestamp < rhs.timestamp; }


//////////////////////////////////////// SensorLog implementation ////////////////////////////////////////////
SensorLog::SensorLog(void)
{
    // Nothing needed for default construction
}


SensorLog::SensorLog(const std::string& logFilename)
: name_(logFilename)
{
    // Load all values from the provided LCM log. The LCM callbacks instance that will interact with LCM and callback
    // to add the values to the sensor log.
    LCMSensorCallbacks callbacks(*this);

    // To create a log provider, we can use a special Url for LCM. The Url to use was found in the source for the
    // lcm-logplayer. The interesting command is setting speed=0, which causes the log to playback as fast as it can
    // be read, as opposed to waiting between messages. Info on speed=0 can be found in lcm/lcm_file.c.
    std::ostringstream logProviderUrl;
    logProviderUrl << "file://" << logFilename << "?speed=0"; // default mode is read mode, but have to read LCM code carefully to actually know that
    system::ModuleCommunicator comm(logProviderUrl.str(), "");

    // Subscribe to each message in the log
    comm.subscribeTo<polar_laser_scan_t>(&callbacks);
    comm.subscribeTo<imu_data_t>(&callbacks);
    comm.subscribeTo<odometry_t>(&callbacks);
    comm.subscribeTo<encoder_data_t>(&callbacks);
    comm.subscribeTo<robot::commanded_joystick_t>(&callbacks);
    comm.subscribeTo<robot::commanded_velocity_t>(&callbacks);

    // Keep reading messages until the processIncoming doesn't return success, indicating that there's no data left
    while(comm.processIncoming() == 0);

    // Once all data has been read, then sort all data by timestamp to ensure correct ordering
    std::sort(frontLaser_.begin(), frontLaser_.end(), sort_by_time<polar_laser_scan_t>);
    std::sort(backLaser_.begin(), backLaser_.end(), sort_by_time<polar_laser_scan_t>);
    std::sort(imu_.begin(), imu_.end(), sort_by_time<imu_data_t>);
    std::sort(odometry_.begin(), odometry_.end(), sort_by_time<odometry_t>);
    std::sort(encoders_.begin(), encoders_.end(), sort_by_time<encoder_data_t>);
    std::sort(joystickCommand_.begin(), joystickCommand_.end(), sort_by_time<robot::commanded_joystick_t>);
    std::sort(velocityCommand_.begin(), velocityCommand_.end(), sort_by_time<robot::commanded_velocity_t>);

    std::cout << "INFO: SensorLog: Loaded the following data from " << name_ << ":\n"
        << "Front laser:" << frontLaser_.size() << '\n'
        << "Back laser: " << backLaser_.size() << '\n'
        << "IMU:        " << imu_.size() << '\n'
        << "Odometry:   " << odometry_.size() << '\n'
        << "Encoders:   " << encoders_.size() << '\n'
        << "Joystick:   " << joystickCommand_.size() << '\n'
        << "Cmd Vel:    " << velocityCommand_.size() << '\n';
}


void SensorLog::addFrontLaser(const polar_laser_scan_t& scan)
{
    frontLaser_.push_back(scan);
}


void SensorLog::addBackLaser(const polar_laser_scan_t& scan)
{
    backLaser_.push_back(scan);
}


void SensorLog::addImu(const imu_data_t& imu)
{
    imu_.push_back(imu);
}


void SensorLog::addOdometry(const odometry_t& odom)
{
    odometry_.push_back(odom);
}


void SensorLog::addEncoders(const encoder_data_t& encoders)
{
    encoders_.push_back(encoders);
}


void SensorLog::addCommandedJoystick(const robot::commanded_joystick_t& joystick)
{
    joystickCommand_.push_back(joystick);
}


void SensorLog::addCommandedVelocity(const robot::commanded_velocity_t& command)
{
    velocityCommand_.push_back(command);
}

} // namespace sensors
} // namespace vulcan
