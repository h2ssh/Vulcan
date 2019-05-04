/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sensor_log_test.cpp
* \author   Collin Johnson
* 
* sensor_log_test is a test program that loads the requested log file and outputs how much data was found inside.
*/

#include <sensors/sensor_log.h>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

using namespace vulcan;

std::string logFilename;

TEST(LogLoadsTest, LogHasData)
{
    sensors::SensorLog log(logFilename);
    
    EXPECT_GT(log.sizeFrontLaser(), std::size_t(0));
    EXPECT_GT(log.sizeBackLaser(), std::size_t(0));
    EXPECT_GT(log.sizeImu(), std::size_t(0));
    EXPECT_GT(log.sizeOdometry(), std::size_t(0));
    EXPECT_GT(log.sizeEncoders(), std::size_t(0));
}

TEST(LogLoadsTest, DataIsSorted)
{
    sensors::SensorLog log(logFilename);
    
    for(std::size_t n = 1; n < log.sizeFrontLaser(); ++n)
    {
        EXPECT_GT((log.beginFrontLaser() + n)->timestamp, (log.beginFrontLaser() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeBackLaser(); ++n)
    {
        EXPECT_GT((log.beginBackLaser() + n)->timestamp, (log.beginBackLaser() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeOdometry(); ++n)
    {
        EXPECT_GT((log.beginOdometry() + n)->timestamp, (log.beginOdometry() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeEncoders(); ++n)
    {
        EXPECT_GT((log.beginEncoders() + n)->timestamp, (log.beginEncoders() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeImu(); ++n)
    {
        EXPECT_GT((log.beginImu() + n)->timestamp, (log.beginImu() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeCommandedJoystick(); ++n)
    {
        EXPECT_GT((log.beginCommandedJoystick() + n)->timestamp, (log.beginCommandedJoystick() + n - 1)->timestamp);
    }
    
    for(std::size_t n = 1; n < log.sizeCommandedVelocity(); ++n)
    {
        EXPECT_GT((log.beginCommandedVelocity() + n)->timestamp, (log.beginCommandedVelocity() + n - 1)->timestamp);
    }
}

TEST(LogAddTest, CanAddFrontLaser)
{
    sensors::SensorLog log;
    log.addFrontLaser(polar_laser_scan_t());
    EXPECT_EQ(log.sizeFrontLaser(), std::size_t(1));
}

TEST(LogAddTest, CanAddBackLaser)
{
    sensors::SensorLog log;
    log.addBackLaser(polar_laser_scan_t());
    EXPECT_EQ(log.sizeBackLaser(), std::size_t(1));
}

TEST(LogAddTest, CanAddImu)
{
    sensors::SensorLog log;
    log.addImu(imu_data_t());
    EXPECT_EQ(log.sizeImu(), std::size_t(1));
}

TEST(LogAddTest, CanAddOdometry)
{
    sensors::SensorLog log;
    log.addOdometry(odometry_t());
    EXPECT_EQ(log.sizeOdometry(), std::size_t(1));
}

TEST(LogAddTest, CanAddEncoders)
{
    sensors::SensorLog log;
    log.addEncoders(encoder_data_t());
    EXPECT_EQ(log.sizeEncoders(), std::size_t(1));
}

TEST(LogAddTest, CanAddCommandedJoystick)
{
    sensors::SensorLog log;
    log.addCommandedJoystick(robot::commanded_joystick_t());
    EXPECT_EQ(log.sizeCommandedJoystick(), std::size_t(1));
}

TEST(LogAddTest, CanAddCommandedVelocity)
{
    sensors::SensorLog log;
    log.addCommandedVelocity(robot::commanded_velocity_t());
    EXPECT_EQ(log.sizeCommandedVelocity(), std::size_t(1));
}


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    
    // Remaining parameters are for my program. This is where the log file can be specified
    assert(argc > 1);
    logFilename = argv[1];
    std::cout << "Reading log:" << logFilename << '\n';
    
    return RUN_ALL_TESTS();
}
