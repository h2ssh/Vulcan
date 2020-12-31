/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "sensors/sensor_log.h"
#include <boost/range/iterator_range.hpp>
#include <fstream>
#include <iostream>
#include <string>

using namespace vulcan;

void save_encoders(const sensors::SensorLog& log, const std::string& filename);
void save_odometry(const sensors::SensorLog& log, const std::string& filename);
void save_imu(const sensors::SensorLog& log, const std::string& filename);
void save_joystick(const sensors::SensorLog& log, const std::string& filename);


int main(int argc, char** argv)
{
    const std::string kEncoderName("encoders_");
    const std::string kOdometryName("odometry_");
    const std::string kIMUName("imu_");
    const std::string kJoystickName("joystick_");

    if (argc < 2) {
        std::cout << "Expected input: motion_sensor_to_text 'log_filename'\n";
        return -1;
    }

    std::string logFilename(argv[1]);

    sensors::SensorLog log(logFilename);

    save_encoders(log, kEncoderName + logFilename);
    save_odometry(log, kOdometryName + logFilename);
    save_imu(log, kIMUName + logFilename);
    save_joystick(log, kJoystickName + logFilename);

    return 0;
}


void save_encoders(const sensors::SensorLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for (auto& encoders : boost::make_iterator_range(log.beginEncoders(), log.endEncoders())) {
        out << encoders.timestamp << ' ' << encoders.deltaLeftWheel << ' ' << encoders.deltaRightWheel << '\n';
    }
}


void save_odometry(const sensors::SensorLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for (auto& odom : boost::make_iterator_range(log.beginOdometry(), log.endOdometry())) {
        out << odom.timestamp << ' ' << odom.x << ' ' << odom.y << ' ' << odom.theta << ' ' << odom.translation << ' '
            << odom.rotation << '\n';
    }
}


void save_imu(const sensors::SensorLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for (auto& imu : boost::make_iterator_range(log.beginImu(), log.endImu())) {
        out << imu.timestamp << ' ' << imu.rotationalVelocity[IMU_YAW_INDEX] << '\n';
    }
}


void save_joystick(const sensors::SensorLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for (auto& joystick : boost::make_iterator_range(log.beginCommandedJoystick(), log.endCommandedJoystick())) {
        out << joystick.timestamp << ' ' << joystick.forward << ' ' << joystick.left << '\n';
    }
}
