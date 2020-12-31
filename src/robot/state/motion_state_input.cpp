/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_state_input.cpp
* \author   Collin Johnson
* 
* Implementation of MotionStateInputQueue.
*/

#include "robot/state/motion_state_input.h"
#include "utils/timestamp.h"
#include "utils/auto_mutex.h"
#include <iostream>

namespace vulcan
{
namespace robot
{
    
template <class T>
void read_queue_into_vector(std::deque<T>& queue, std::vector<T>& vector)
{
    std::copy(queue.begin(), queue.end(), std::back_inserter(vector));
    queue.clear();
    std::sort(vector.begin(), vector.end(), [](const T& lhs, const T& rhs) {
        return lhs.timestamp <= rhs.timestamp;
    });
}


MotionStateInputQueue::MotionStateInputQueue(void)
{
    // Odometry data is alwasy required.
    odometry_.isReceiving = true;
}


bool MotionStateInputQueue::isDataAvailable(void) const
{
    return odometry_.isReady() && imu_.isReady() && encoders_.isReady() && joystick_.isReady() && command_.isReady();
}


motion_state_input_t MotionStateInputQueue::readInput(void)
{
    utils::AutoMutex autoLock(dataLock_);
    
    motion_state_input_t input;
    
    read_queue_into_vector(imuQueue_,               input.imu);
    read_queue_into_vector(odometryQueue_,          input.odometry);
    read_queue_into_vector(encodersQueue_,          input.encoders);
    read_queue_into_vector(commandedJoystickQueue_, input.commandedJoysticks);
    read_queue_into_vector(commandedVelocityQueue_, input.commandedVelocities);
    read_queue_into_vector(poseQueue_,              input.localizedPoses);
    
    odometry_.hasData = false;
    encoders_.hasData = false;
    joystick_.hasData = false;
    command_.hasData = false;
    imu_.hasData = false;
    
    return input;
}


void MotionStateInputQueue::handleData(const imu_data_t& imu, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    imuQueue_.push_back(imu);
    imu_.isReceiving = true;
    imu_.hasData = true;
}


void MotionStateInputQueue::handleData(const odometry_t& odometry, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    odometryQueue_.push_back(odometry);
    odometry_.hasData = true;
}


void MotionStateInputQueue::handleData(const encoder_data_t& encoders, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    encodersQueue_.push_back(encoders);
    encoders_.isReceiving = true;
    encoders_.hasData = true;
}


void MotionStateInputQueue::handleData(const commanded_joystick_t& commandedJoystick, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    commandedJoystickQueue_.push_back(commandedJoystick);
    joystick_.isReceiving = true;
    joystick_.hasData = true;
}

void MotionStateInputQueue::handleData(const commanded_velocity_t& commandedVelocity, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    commandedVelocityQueue_.push_back(commandedVelocity);
    command_.isReceiving = true;
    command_.hasData = true;
}


void MotionStateInputQueue::handleData(const pose_distribution_t& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    poseQueue_.push_back(pose);
}

} // namespace robot
} // namespace vulcan
