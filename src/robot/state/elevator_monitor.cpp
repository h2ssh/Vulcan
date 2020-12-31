/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     elevator_monitor.cpp
* \author   Collin Johnson
*
* Definition of ElevatorMonitor.
*/

#include "robot/state/elevator_monitor.h"
#include "core/imu_data.h"
#include "utils/auto_mutex.h"
#include "utils/config_file.h"
#include "utils/timestamp.h"
#include <map>
#include <algorithm>
#include <iostream>
#include <numeric>

// #define DEBUG_STATS
// #define DEBUG_STATE_CHANGE

namespace vulcan
{
namespace robot
{
    
const std::string kElevatorHeading  ("ElevatorMonitorParameters");
const std::string kMotionDurationKey("motion_check_duration_ms");
const std::string kQueueDurationKey ("queue_duration_ms");
const std::string kStoppedXVarKey   ("x_variance_when_stopped");
const std::string kStoppedYVarKey   ("y_variance_when_stopped");
const std::string kStoppedZVarKey   ("z_variance_when_stopped");
const std::string kMinAccelMagKey   ("min_elevator_accel_magnitude");
const std::string kMinAccelTimeKey  ("min_elevator_accel_duration_ms");
    

double mean(std::deque<double>::iterator begin, std::deque<double>::iterator end);
double variance(std::deque<double>::iterator begin, std::deque<double>::iterator end);

    
elevator_monitor_params_t::elevator_monitor_params_t(const utils::ConfigFile& config)
: motionCheckIntervalUs(config.getValueAsInt32(kElevatorHeading, kMotionDurationKey)*1000)
, xVarianceWhenStopped(config.getValueAsDouble(kElevatorHeading, kStoppedXVarKey))
, yVarianceWhenStopped(config.getValueAsDouble(kElevatorHeading, kStoppedYVarKey))
, zVarianceWhenStopped(config.getValueAsDouble(kElevatorHeading, kStoppedZVarKey))
, minElevatorAccelMagnitude(config.getValueAsDouble(kElevatorHeading, kMinAccelMagKey))
, minElevatorAccelDurationUs(config.getValueAsInt32(kElevatorHeading, kMinAccelTimeKey)*1000)
, durationOfQueueUs(config.getValueAsInt32(kElevatorHeading, kQueueDurationKey)*1000)
{
}


ElevatorMonitor::ElevatorMonitor(const elevator_monitor_params_t& params)
: haveNewElevatorState(false)
, gravityEstimate(-9.81)
, params(params)
{
}


void ElevatorMonitor::initialize(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<imu_data_t>(this);
}


void ElevatorMonitor::estimate(system::ModuleCommunicator& transmitter)
{
    // Don't calculate stats if there aren't enough data yet
    if(numMeasurements > 1)
    {
        {
            utils::AutoMutex autoLock(dataLock);
            calculateIMUStatistics();
            determineElevatorState();
        }
        
        transmitter.sendMessage(elevator);
            
#ifdef DEBUG_STATS
        std::cout<<"DEBUG:ElevatorMonitor: x:("<<xAxis.mean<<','<<xAxis.variance
                 <<") y:("<<yAxis.mean<<','<<yAxis.variance
                 <<") z:("<<zAxis.mean<<','<<zAxis.variance<<")\n";
#endif
    }
}


void ElevatorMonitor::handleData(const imu_data_t& imu, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);
    enqueueData(imu);
    removeOldData();
    numMeasurements = timestamps.size();
}


void ElevatorMonitor::enqueueData(const imu_data_t& imu)
{
    timestamps.push_back(imu.timestamp);
    xAxis.accels.push_back(imu.acceleration[IMU_X_INDEX]);
    yAxis.accels.push_back(imu.acceleration[IMU_Y_INDEX]);
    zAxis.accels.push_back(imu.acceleration[IMU_Z_INDEX]);
}


void ElevatorMonitor::removeOldData(void)
{
    // Then erase all data occurring before the index of the queue duration
    int n = findStartIndexOfInterval(params.durationOfQueueUs);

    if(n > 0)
    {
        timestamps.erase(timestamps.begin(), timestamps.begin()+n);
        xAxis.accels.erase(xAxis.accels.begin(), xAxis.accels.begin()+n);
        yAxis.accels.erase(yAxis.accels.begin(), yAxis.accels.begin()+n);
        zAxis.accels.erase(zAxis.accels.begin(), zAxis.accels.begin()+n);
    }
}


void ElevatorMonitor::calculateIMUStatistics(void)
{
    int statsStartIndex = findStartIndexOfInterval(params.motionCheckIntervalUs);
    calculateAxisStatistics(statsStartIndex, xAxis);
    calculateAxisStatistics(statsStartIndex, yAxis);
    calculateAxisStatistics(statsStartIndex, zAxis);
    estimateGravity(statsStartIndex);

    timeDiffs.resize(timestamps.size());
    diffFromGravity.resize(zAxis.accels.size());

    std::adjacent_difference(timestamps.begin(), timestamps.end(), timeDiffs.begin());

    auto subtractGravityFunc = [this](double accel) { return accel - this->gravityEstimate; };

    std::transform(zAxis.accels.begin(), zAxis.accels.end(), diffFromGravity.begin(), subtractGravityFunc);
}


int ElevatorMonitor::findStartIndexOfInterval(int64_t duration)
{
    // Search from the most recent timestamp into the past to find the first timestamp for which
    // the time range of data is greater than the duration argument.
    int n = timestamps.size();

    while(--n >= 0)
    {
        if(timestamps.back() - timestamps[n] > duration)
        {
            break;
        }
    }

    return (n < 0) ? 0 : n;
}


int ElevatorMonitor::findIndexWithTime(int64_t time)
{
    for(int n = timestamps.size(); --n >= 0;)
    {
        if(timestamps[n] == time)
        {
            return n;
        }
    }

    std::cerr<<"ERROR:ElevatorMonitor: Failed to find measurement with start time:"<<time<<'\n';

    return -1;
}


void ElevatorMonitor::calculateAxisStatistics(int startIndex, imu_axis_t& axis)
{
    axis.mean     = mean(axis.accels.begin() + startIndex, axis.accels.end());
    axis.variance = variance(axis.accels.begin() + startIndex, axis.accels.end());
}


void ElevatorMonitor::estimateGravity(std::size_t startIndex)
{
    // Only update the gravity estimate when the z-variance is low, indicating little or no motion on the axis
    // If already on the elevator, also don't estimate because who knows if the elevator is just applying a very
    // constant acceleration or not
    if((zAxis.variance > params.zVarianceWhenStopped) || (detectionState != ROBOT_STOPPED))
    {
        return;
    }

    double sum = 0.0;

    for(std::size_t n = startIndex; n < zAxis.accels.size(); ++n)
    {
        sum += zAxis.accels[n];
    }

    if(startIndex < zAxis.accels.size())
    {
        double newGravity = sum / (zAxis.accels.size() - startIndex);

        // Don't ever allow gravity to change by too much
        if(std::abs(newGravity - gravityEstimate) < 0.05)
        {
            gravityEstimate = newGravity;
        }
    }
}


void ElevatorMonitor::determineElevatorState(void)
{
    switch(detectionState)
    {
    case ROBOT_MOVING:
        if(isElevatorStarting())
        {
            detectionState = ELEVATOR_MOVING;
            initializeElevatorMotion();
        }
        else if(!isRobotMoving())
        {
            detectionState = ROBOT_STOPPED;
        }
        break;

    case ROBOT_STOPPED:
        if(isElevatorStarting())
        {
            detectionState = ELEVATOR_MOVING;
            initializeElevatorMotion();
        }
        else if(isRobotMoving())
        {
            detectionState = ROBOT_MOVING;
        }
        break;

    case ELEVATOR_MOVING:
        processElevatorMoving();
        break;
    }
}


bool ElevatorMonitor::isRobotMoving(void) const
{
    return (xAxis.variance > params.xVarianceWhenStopped) ||
           (yAxis.variance > params.yVarianceWhenStopped);
}


bool ElevatorMonitor::isElevatorStarting(void)
{
    if(zAxis.variance < params.zVarianceWhenStopped)
    {
        return false;
    }

    accel_t accel = findElevatorAccel();

    if(accel.first > 0)
    {
        elevatorStartTime    = lastIntegrationTime = accel.first;
        elevatorStopTime     = 0;
        hasElevatorSlowed    = false;
        initialElevatorAccel = accel.second;
        return true;
    }

    return false;
}


bool ElevatorMonitor::isElevatorStopped(void)
{
    if(!hasElevatorSlowed)
    {
        accel_t accel = findElevatorAccel();

        // If an acceleration in the opposite direction has occurred, then stopping has begun
        hasElevatorSlowed = (accel.first > 0) && (accel.second * initialElevatorAccel < 0.0) && (accel.first != elevatorStartTime);

        if(hasElevatorSlowed)
        {
            std::cout<<"Elevator slowed:"<<accel.first<<'\n';
        }
    }
    else // if(hasElevatorSlowed)
    {
//         std::cout<<"z-var:"<<zAxis.variance<<'\n';
        return zAxis.variance < params.zVarianceWhenStopped;
    }

    return false;
}


ElevatorMonitor::accel_t ElevatorMonitor::findElevatorAccel(void)
{
    int64_t motionStart = 0;
    double  totalAccel  = 0.0;
    int     numAccel    = 0;

    int64_t timeWithSign = 0;

    // Skip the first value because adjacent_difference is just the first value -- need to initialize it somehow!
    for(std::size_t n = 1; n < diffFromGravity.size(); ++n)
    {
        int sign = 0;

        if(diffFromGravity[n] < -0.01)
        {
            sign = -1;
        }
        else if(diffFromGravity[n] > 0.01)
        {
            sign = 1;
        }
        else
        {
            timeWithSign = 0;
        }

        int64_t lastTime = timeWithSign;

        timeWithSign += sign * timeDiffs[n];

        if(lastTime == 0 && timeWithSign != 0) // sign flipped!
        {
            motionStart = timestamps[n];
            numAccel    = 0;
            totalAccel  = 0.0;
        }

        ++numAccel;
        totalAccel += diffFromGravity[n];

        // As soon as an acceleration has occurred of sufficient magnitude and duration, then
        if((std::abs(timeWithSign) > params.minElevatorAccelDurationUs) && (std::abs(diffFromGravity[n]) > params.minElevatorAccelMagnitude))
        {
//             std::cout<<"Detected elevator motion: Time:"<<timeWithSign<<" Duration:"<<(timestamps[n] - motionStart)<<" Magnitude:"<<totalAccel/numAccel<<'\n';
            return std::make_pair(motionStart, totalAccel/numAccel);
        }

//         if((std::abs(diffFromGravity[n]) > params.minElevatorAccelMagnitude))
//         {
//             std::cout<<"Time:"<<timeWithSign<<" Accel:"<<totalAccel/numAccel<<" Diff:"<<diffFromGravity[n]<<'\n';
//         }
    }

    return std::make_pair(0, 0.0);
}


void ElevatorMonitor::initializeElevatorMotion(void)
{
    elevatorDistance = 0.0;
    elevatorVelocity = 0.0;
}


void ElevatorMonitor::processElevatorMoving(void)
{
    // First check if stopped because
    if(isElevatorStopped())
    {
        detectionState   = isRobotMoving() ? ROBOT_MOVING : ROBOT_STOPPED;
        elevatorStopTime = timestamps.back();

        std::cout<<"Elevator stopped:"<<timestamps.back()<<'\n';
    }

    integrateNewData();
    updateElevatorState();

    if(detectionState != ELEVATOR_MOVING)
    {
        timestamps.clear();
        timeDiffs.clear();
        diffFromGravity.clear();
        xAxis.accels.clear();
        yAxis.accels.clear();
        zAxis.accels.clear();
    }
}


void ElevatorMonitor::integrateNewData(void)
{
    int integrationStart = findIndexWithTime(lastIntegrationTime) + 1;
    int integrationEnd   = (elevatorStopTime > elevatorStartTime) ? findIndexWithTime(elevatorStopTime) : zAxis.accels.size();

    for(int n = integrationStart; n < integrationEnd; ++n)
    {
        double timeDelta = utils::usec_to_sec(timeDiffs[n]);
        double accel     = diffFromGravity[n];

        elevatorDistance += accel*timeDelta*timeDelta + elevatorVelocity*timeDelta;
        elevatorVelocity += accel*timeDelta;
    }

    // If the robot has stopped, then the integration will have stopped anyhow, so no need to worry about this value might
    // otherwise be.
    lastIntegrationTime = timestamps.back();

    std::cout<<"Current elevator motion: Dist:"<<elevatorDistance<<" Vel:"<<elevatorVelocity<<" Gravity:"<<gravityEstimate<<'\n';
}


void ElevatorMonitor::updateElevatorState(void)
{
    // this elevator value incorporates all data up to the current time, regardless of what the elevator what doing during the interval
    elevator.timestamp = timestamps.back();

    if(detectionState == ELEVATOR_MOVING)
    {
        elevator.state = (elevatorVelocity > 0) ? ELEVATOR_MOVING_UP : ELEVATOR_MOVING_DOWN;
    }
    else
    {
        elevator.state = ELEVATOR_STOPPED;
    }

    elevator.startTime = elevatorStartTime;
    elevator.stopTime  = elevatorStopTime;

    elevator.distance = elevatorDistance;
    elevator.velocity = elevatorVelocity;

    haveNewElevatorState = true;
}


double mean(std::deque<double>::iterator begin, std::deque<double>::iterator end)
{
    double sum = 0;
    int    n   = 0;

    while(begin != end)
    {
        sum += *begin;
        ++n;
        ++begin;
    }

    if(n > 0)
    {
        return sum / n;
    }
    else
    {
        return sum;
    }
}


double variance(std::deque<double>::iterator begin, std::deque<double>::iterator end)
{
    double avg = mean(begin, end);

    double sum   = 0;
    double error = 0;
    int    n     = 0;

    while(begin != end)
    {
        sum   += (*begin-avg) * (*begin-avg);
        error += *begin - avg;

        ++n;
        ++begin;
    }

    if(n > 1)
    {
        return (sum + error*error/n) / (n - 1);
    }
    else
    {
        return sum;
    }
}

} // namespace robot
} // namespace vulcan
