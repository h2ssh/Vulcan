/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data_logger.h
 * \author   Collin Johnson
 *
 * Definition of DataLogger policy class.
 */

#ifndef LOGGING_LOGGER_DATA_LOGGER_H
#define LOGGING_LOGGER_DATA_LOGGER_H

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{

struct encoder_data_t;
struct imu_data_t;
struct motion_state_t;
struct polar_laser_scan_t;

namespace robot
{
struct commanded_joystick_t;
}
namespace robot
{
struct motion_command_t;
}
namespace hssh
{
class LocalPerceptualMap;
}
namespace tracker
{
class DynamicObjectCollection;
}

namespace logging
{

/**
 * DataLogger logs many different types of Vulcan data into text format for easy use in Matlab. The file
 * names and formats are detailed below:
 *
 *   1) basename_objects.log
 *   2) basename_velocity.log
 *   3) basename_pose.log
 *   4) basename_imu.log
 *   5) basename_trajectory_count.log
 *   6) basename_LASER_CHANNEL.log  (where LASER_CHANNEL will be FRONT, BACK, etc)
 *   7) basename_lpm.log (only the current LPM is stored at any particular time, rather than a long series of them)
 *   8) basename_encoders.log
 *
 * The format for these logs is:
 *
 *   1) time object_id x y vel_x vel_y
 *   2) time vel_x vel_y
 *   3) time x y theta
 *   4) time accel_x accel_y accel_z omega_yaw omega_pitch omega_roll yaw pitch roll
 *   5) num_trajectories_evaluated
 *   6) See laser/laser_io.h for the format
 *   7) See hssh/local_metric/lpm_io.h for the format
 *   8) time id leftIndexPulseTotal rightIndexPulseTotal leftTicksTotal rightTicksTotal
 */
class DataLogger
{
public:
    /**
     * Constructor for DataLogger.
     *
     * \param    basename        Base filename for the logs
     */
    DataLogger(const std::string& basename);

    template <typename T>
    void handleData(const T& data, const std::string& channel)
    {
        log(data, channel);
    }

private:
    void log(const polar_laser_scan_t& scan, const std::string& channel);
    void log(const tracker::DynamicObjectCollection& objects, const std::string& channel);
    void log(const motion_state_t& state, const std::string& channel);
    void log(const imu_data_t& imu, const std::string& channel);
    void log(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void log(const encoder_data_t& encoders, const std::string& channel);
    void log(const robot::motion_command_t& command, const std::string& channel);
    void log(const robot::commanded_joystick_t& command, const std::string& channel);

    std::string basename;

    std::ofstream objectLog;
    std::ofstream velocityLog;
    std::ofstream poseLog;
    std::ofstream imuLog;
    std::ofstream trajectoryCountLog;
    std::ofstream lpmLog;
    std::ofstream encoderLog;
    std::ofstream joystickCmdLog;
    std::ofstream commandedJoystickLog;

    std::vector<std::unique_ptr<std::ofstream>> laserLogs;
    std::vector<std::string> laserChannels;
};

}   // namespace logging
}   // namespace vulcan

#endif   // LOGGING_LOGGER_DATA_LOGGER_H
