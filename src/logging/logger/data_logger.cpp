/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data_logger.cpp
 * \author   Collin Johnson
 *
 * Definition of DataLogger.
 */

#include "logging/logger/data_logger.h"
#include "core/imu_data.h"
#include "core/motion_state.h"
#include "core/odometry.h"
#include "hssh/local_metric/lpm_io.h"
#include "laser/laser_io.h"
#include "robot/commands.h"
#include "tracker/dynamic_object_collection.h"
#include <cstdlib>
#include <iostream>

namespace vulcan
{
namespace logging
{

DataLogger::DataLogger(const std::string& basename)
: basename(basename)
, objectLog(basename + "_objects.log")
, velocityLog(basename + "_velocity.log")
, poseLog(basename + "_pose.log")
, imuLog(basename + "_imu.log")
, trajectoryCountLog(basename + "_trajectory_count.log", std::ios_base::app)
, lpmLog(basename + "_lpm.log")
, encoderLog(basename + "_encoders.log")
, joystickCmdLog(basename + "_joystick_command.log")
, commandedJoystickLog(basename + "_commanded_joystick.log")
{
}


void DataLogger::log(const polar_laser_scan_t& scan, const std::string& channel)
{
    for (size_t n = 0; n < laserLogs.size(); ++n) {
        if (laserChannels[n] == channel) {
            laser::save_laser_scan_to_file(scan, *laserLogs[n]);
            return;
        }
    }

    laserChannels.push_back(channel);
    laserLogs.emplace_back(new std::ofstream(basename + channel + ".log"));
    laser::save_laser_scan_to_file(scan, *laserLogs.back());
}


void DataLogger::log(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
    for (auto& object : objects) {
        objectLog << object->timeLastSeen() << ' ' << object->position().x << ' ' << object->position().y << ' '
                  << object->velocity().x << ' ' << object->velocity().y << '\n';
    }
}


void DataLogger::log(const motion_state_t& state, const std::string& channel)
{
    velocityLog << state.timestamp << ' ' << state.velocity.linear << ' ' << state.velocity.angular << std::endl;
    poseLog << state.timestamp << ' ' << state.pose.x << ' ' << state.pose.y << ' ' << state.pose.theta << std::endl;
}


void DataLogger::log(const imu_data_t& imu, const std::string& channel)
{
    imuLog << imu.timestamp << ' ' << imu.acceleration[IMU_X_INDEX] << ' ' << imu.acceleration[IMU_Y_INDEX] << ' '
           << imu.acceleration[IMU_Z_INDEX] << ' ' << imu.rotationalVelocity[IMU_YAW_INDEX] << ' '
           << imu.rotationalVelocity[IMU_PITCH_INDEX] << ' ' << imu.rotationalVelocity[IMU_ROLL_INDEX] << ' '
           << imu.orientation[IMU_YAW_INDEX] << ' ' << imu.orientation[IMU_PITCH_INDEX] << ' '
           << imu.orientation[IMU_ROLL_INDEX] << std::endl;
}


void DataLogger::log(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    hssh::save_lpm_occupancy_text(lpm, lpmLog);
}


void DataLogger::log(const encoder_data_t& encoders, const std::string& channel)
{
    encoderLog << encoders.timestamp << ' ' << encoders.id << ' ' << encoders.leftIndexPulseTotal << ' '
               << encoders.rightIndexPulseTotal << ' ' << encoders.leftTicksTotal << ' ' << encoders.rightTicksTotal
               << '\n';
}


void DataLogger::log(const robot::motion_command_t& command, const std::string& channel)
{
    joystickCmdLog << command.joystickCommand.timestamp << ' ' << command.joystickCommand.forward << ' '
                   << command.joystickCommand.left << '\n';
}


void DataLogger::log(const robot::commanded_joystick_t& command, const std::string& channel)
{
    commandedJoystickLog << command.timestamp << ' ' << command.forward << ' ' << command.left << '\n';
}

}   // namespace logging
}   // namespace vulcan
