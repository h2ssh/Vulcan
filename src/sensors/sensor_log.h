/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sensor_log.h
* \author   Collin Johnson
*
* Declaration of SensorLog.
*/

#ifndef SENSORS_SENSOR_LOG_H
#define SENSORS_SENSOR_LOG_H

#include "core/laser_scan.h"
#include "robot/commands.h"
#include "core/imu_data.h"
#include "core/odometry.h"

namespace vulcan
{
namespace sensors
{

/**
* SensorLog is a utility for loading the contents of a sensor log into RAM. SensorLog exists to avoid needing to use
* a logplayer to read all the sensor data from a log for processing, be it for doing calculations or converting to a
* Matlab-friendly format.
*
* The data in SensorLog is sorted in order of ascending timestamps, so begin->timestamp < end->timestamp.
*
* A SensorLog instance is created by providing an LCM log. This log will be read and converted into the appropriate data
* types. The various types can then be iterated over as desired.
*
* IMPORTANT: Calling any of the addXXXX methods will invalidate iterators of the same type. In general, you should just
* provide the log file to the constructor and never need to touch the add methods.
*/
class SensorLog
{
public:

    using laser_iterator = std::vector<polar_laser_scan_t>::const_iterator;
    using imu_iterator = std::vector<imu_data_t>::const_iterator;
    using odom_iterator = std::vector<odometry_t>::const_iterator;
    using encoder_iterator = std::vector<encoder_data_t>::const_iterator;
    using joystick_iterator = std::vector<robot::commanded_joystick_t>::const_iterator;
    using command_iterator = std::vector<robot::commanded_velocity_t>::const_iterator;

    /**
    * Default constructor for SensorLog.
    *
    * Create an empty log.
    */
    SensorLog(void);

    /**
    * Constructor for SensorLog.
    *
    * Create a SensorLog containing all data in the provided LCM log.
    *
    * \param    logFilename         Name of the log to be loaded
    */
    SensorLog(const std::string& logFilename);

    // Disable copying, because these logs can be huge. Only moves are allowed for sanity's sake
    SensorLog(const SensorLog&) = delete;
    SensorLog& operator=(const SensorLog&) = delete;

    SensorLog(SensorLog&&) = default;
    SensorLog& operator=(SensorLog&&) = default;

    /**
    * name retrieves the name of the sensor log. It is just the end of the filename, not the full path.
    */
    std::string name(void) const { return name_; }


    // Iterator access for the data in the log
    std::size_t sizeFrontLaser(void) const { return frontLaser_.size(); }
    laser_iterator beginFrontLaser(void) const { return frontLaser_.begin(); }
    laser_iterator endFrontLaser(void) const { return frontLaser_.end(); }

    std::size_t sizeBackLaser(void) const { return backLaser_.size(); }
    laser_iterator beginBackLaser(void) const { return backLaser_.begin(); }
    laser_iterator endBackLaser(void) const { return backLaser_.end(); }

    std::size_t sizeImu(void) const { return imu_.size(); }
    imu_iterator beginImu(void) const { return imu_.begin(); }
    imu_iterator endImu(void) const { return imu_.end(); }

    std::size_t sizeOdometry(void) const { return odometry_.size(); }
    odom_iterator beginOdometry(void) const { return odometry_.begin(); }
    odom_iterator endOdometry(void) const { return odometry_.end(); }

    std::size_t sizeEncoders(void) const { return encoders_.size(); }
    encoder_iterator beginEncoders(void) const { return encoders_.begin(); }
    encoder_iterator endEncoders(void) const { return encoders_.end(); }

    std::size_t sizeCommandedJoystick(void) const { return joystickCommand_.size(); }
    joystick_iterator beginCommandedJoystick(void) const { return joystickCommand_.begin(); }
    joystick_iterator endCommandedJoystick(void) const { return joystickCommand_.end(); }

    std::size_t sizeCommandedVelocity(void) const { return velocityCommand_.size(); }
    command_iterator beginCommandedVelocity(void) const { return velocityCommand_.begin(); }
    command_iterator endCommandedVelocity(void) const { return velocityCommand_.end(); }

    // Add the various type of data to the log
    void addFrontLaser(const polar_laser_scan_t& scan);
    void addBackLaser(const polar_laser_scan_t& scan);
    void addImu(const imu_data_t& imu);
    void addOdometry(const odometry_t& odom);
    void addEncoders(const encoder_data_t& encoders);
    void addCommandedJoystick(const robot::commanded_joystick_t& joystick);
    void addCommandedVelocity(const robot::commanded_velocity_t& command);

private:

    std::string name_;

    std::vector<polar_laser_scan_t> frontLaser_;
    std::vector<polar_laser_scan_t> backLaser_;
    std::vector<imu_data_t> imu_;
    std::vector<odometry_t> odometry_;
    std::vector<encoder_data_t> encoders_;
    std::vector<robot::commanded_joystick_t> joystickCommand_;
    std::vector<robot::commanded_velocity_t> velocityCommand_;
};

} // namespace sensors
} // namespace vulcan

#endif // SENSORS_SENSOR_LOG_H
