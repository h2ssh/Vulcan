/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     data_queue.h
* \author   Collin Johnson
* 
* Declaration of MetricSLAMDataQueue.
*/

#ifndef HSSH_UTILS_METRICAL_DATA_QUEUE_H
#define HSSH_UTILS_METRICAL_DATA_QUEUE_H

#include <core/imu_data.h>
#include <core/odometry.h>
#include <core/laser_scan.h>
#include <core/motion_state.h>
#include <utils/mutex.h>
#include <utils/condition_variable.h>
#include <cstdint>
#include <atomic>
#include <deque>
#include <map>

namespace vulcan
{
namespace system { class ModuleCommunicator; }
namespace hssh
{
    
struct metric_slam_data_t;

/**
* SensorInputQueue handles synchronization off all data inputs used for metric SLAM.
*
* Data is synchronized based on laser scans because they are required by both the localization and the occupancy grid
* mapper and also have the slowest update rate of the data used by the LPM. The queue tries to maintain a desired update
* rate.
*
* Each chunk of metric_slam_data_t contains data such that all IMU and odometry timestamps that fall in the interval
* [previousScan.timestamp, currentScan.timestamp]. The queue will wait until an IMU and odometry measurement beyond the
* last laser timestamp has arrived. The odometry and IMU data are linearly interpolated to generate a measurement at
* the exact laser timestamp.
*
* The scanLines are calculated within the queue and don't need to be calculated externally.
*
* To use every scan, set targetUpdateHz to 0.
*/
class MetricSLAMDataQueue
{
public:

    /**
    * Constructor for MetricSLAMDataQueue.
    *
    * If targetUpdateHz == 0.0, then every scan that has enough odometry data will be used.
    * 
    * \param    targetUpdateHz          Desired update rate for the queue
    */
    explicit MetricSLAMDataQueue(double targetUpdateHz);
    
    /**
    * subscribeToData
    */
    void subscribeToData(system::ModuleCommunicator& communicator);

    /**
    * waitForData pauses the 
    */
    bool waitForData(void);

    /**
    * readData
    */
    metric_slam_data_t readData(void);
    
    // Data handlers for subscriptions
    void handleData(const imu_data_t&       imu,      const std::string& channel);
    void handleData(const odometry_t&       odometry, const std::string& channel);
    void handleData(const polar_laser_scan_t& scan,     const std::string& channel);
    void handleData(const motion_state_t&     motion,   const std::string& channel);

private:

    using LaserQueue = std::deque<polar_laser_scan_t>;
    
    std::deque<imu_data_t> imuQueue_;
    std::deque<odometry_t> odometryQueue_;
    std::map<int32_t, LaserQueue> laserQueues_;
    
    // INVARIANT: front of odometry and imu queues have the same timestamp (if receiving IMU)
    
    bool motionQueuesAreSynced_;  // imu and odometry queues
    
    std::atomic<bool> isReceivingIMU_;
    int32_t imuSequenceOffset_;         ///< Amount to add to each incoming IMU sequence number due to interpolation
                                        ///< of IMU values

    velocity_t         velocity_;
    polar_laser_scan_t laserData_;
    
    std::deque<int32_t> laserIds_;          // Ids of lasers from which data has been received
    std::size_t         activeLaserIndex_;  // Index of the next laser to use
    
    int64_t laserTimestamp_;
    int64_t previousEndTime_;
    
    int64_t targetUpdatePeriodUs_;
    
    utils::Mutex             queueLock_;
    utils::ConditionVariable dataAvailableCondition_;

    bool sensorInputHasArrived(void);
    bool hasMotionDataForTime(int64_t time) const;
    void updateNextLaserData(void);
    void moveQueuedDataToSensorInput(metric_slam_data_t& data, int64_t endTime);
    void changeActiveLaserIfNeeded  (void);
    void synchronizeMotionDataQueues(void);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_UTILS_METRICAL_DATA_QUEUE_H
