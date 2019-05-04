/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_state_input.h
* \author   Collin Johnson
* 
* Defintion of motion_state_input_t and MotionStateInputQueue for handling the input from other
* modules being used by the MotionStateEstimator.
*/

#ifndef ROBOT_STATE_MOTION_STATE_INPUT_H
#define ROBOT_STATE_MOTION_STATE_INPUT_H

#include <core/imu_data.h>
#include <core/odometry.h>
#include <sensors/wheel_encoders.h>
#include <robot/commands.h>
#include <core/pose_distribution.h>
#include <utils/mutex.h>
#include <atomic>
#include <deque>
#include <vector>

namespace vulcan
{
namespace robot
{

struct motion_state_input_t
{
    std::vector<imu_data_t>     imu;
    std::vector<odometry_t>     odometry;
    std::vector<encoder_data_t> encoders;
    std::vector<commanded_joystick_t>    commandedJoysticks;
    std::vector<commanded_velocity_t>    commandedVelocities;
    std::vector<pose_distribution_t>     localizedPoses;
};

/**
* MotionStateInputQueue conglomerates all inputs for the MotionStateEstimator.
*/
class MotionStateInputQueue
{
public:
    
    static const uint8_t kImu         = 0x01;
    static const uint8_t kOdometry    = 0x02;
    static const uint8_t kEncoders    = 0x04;
    static const uint8_t kCommandedJs = 0x08;
    static const uint8_t kCommanedeVs = 0x10;
    static const uint8_t kPoses       = 0x20;

    /**
    * Constructor for MotionStateInputQueue.
    */
    MotionStateInputQueue(void);

    /**
    * isDataAvailable checks to see if the specified data types are currently available.
    */
    bool isDataAvailable(void) const;
    
    /**
    * readInput pulls all data from the current queue and places it in a motion_state_input_t structure.
    * 
    * \return   All available sensor data.
    */
    motion_state_input_t readInput(void);
    
    // Data handlers for the DataProducer
    void handleData(const imu_data_t&     imu,               const std::string& channel);
    void handleData(const odometry_t&     odometry,          const std::string& channel);
    void handleData(const encoder_data_t& encoders,          const std::string& channel);
    void handleData(const commanded_joystick_t&    commandedJoystick, const std::string& channel);
    void handleData(const commanded_velocity_t&    commandedVelocity, const std::string& channel);
    void handleData(const pose_distribution_t&     pose,              const std::string& channel);
    
private:

    struct data_status_t
    {
        std::atomic<bool> isReceiving;
        std::atomic<bool> hasData;

        bool isReady(void) const { return !isReceiving || hasData; }

        data_status_t(void)
        : isReceiving(false)
        , hasData(false)
        {
        }
    };
   
    std::deque<imu_data_t>     imuQueue_;
    std::deque<odometry_t>     odometryQueue_;
    std::deque<encoder_data_t> encodersQueue_;
    std::deque<commanded_joystick_t>    commandedJoystickQueue_;
    std::deque<commanded_velocity_t>    commandedVelocityQueue_;
    std::deque<pose_distribution_t>     poseQueue_;
    
    data_status_t odometry_;
    data_status_t encoders_;
    data_status_t imu_;
    data_status_t joystick_;
    data_status_t command_;
    utils::Mutex  dataLock_;
};

}
}

#endif // ROBOT_STATE_MOTION_STATE_INPUT_QUEUE_H
