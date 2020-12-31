/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data_queue.cpp
 * \author   Collin Johnson
 *
 * Definition of MetricSLAMDataQueue.
 */

#include "hssh/metrical/data_queue.h"
#include "hssh/metrical/data.h"
#include "laser/line_extraction.h"
#include "laser/line_extractor_params.h"
#include "system/module_communicator.h"
#include "utils/algorithm_ext.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

template <typename T>
bool has_data_before_time(const std::deque<T>& data, int64_t timestamp);

template <typename T>
bool has_data_after_time(const std::deque<T>& data, int64_t timestamp);

template <typename T>
std::pair<std::size_t, std::size_t> indices_between(const std::deque<T>& data, int64_t startTime, int64_t endTime);

template <typename T>
void move_sensor_data(std::vector<T>& to, std::deque<T>& from, int64_t startTime, int64_t endTime);

laser::laser_scan_lines_t calculate_scan_lines(const polar_laser_scan_t& scan);


MetricSLAMDataQueue::MetricSLAMDataQueue(double targetUpdateHz)
: motionQueuesAreSynced_(false)
, isReceivingIMU_(false)
, imuSequenceOffset_(0)
, activeLaserIndex_(0)
, laserTimestamp_(0)
, previousEndTime_(1)
, dataAvailableCondition_(false)
{
    targetUpdatePeriodUs_ = (targetUpdateHz > 0.0) ? utils::sec_to_usec(1.0 / targetUpdateHz) : 0;
}


void MetricSLAMDataQueue::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<imu_data_t>(this);
    communicator.subscribeTo<odometry_t>(this);
    communicator.subscribeTo<polar_laser_scan_t>(this);
    communicator.subscribeTo<motion_state_t>(this);
}


void MetricSLAMDataQueue::handleData(const imu_data_t& imu, const std::string& channel)
{
    utils::AutoMutex autoMutex(queueLock_);

    if (imuQueue_.empty() || (imu.timestamp > imuQueue_.back().timestamp)) {
        imuQueue_.push_back(imu);
        imuQueue_.back().sequenceNumber += imuSequenceOffset_;
    } else {
        std::cerr << "WARNING:MetricSLAMDataQueue: Ignored IMU data with time in the past.\n";
    }

    isReceivingIMU_ = true;

    if (!motionQueuesAreSynced_ && !odometryQueue_.empty()) {
        synchronizeMotionDataQueues();
    }

    updateNextLaserData();
    dataAvailableCondition_.setPredicate(sensorInputHasArrived());
    dataAvailableCondition_.broadcast();
}


void MetricSLAMDataQueue::handleData(const odometry_t& odometry, const std::string& channel)
{
    utils::AutoMutex autoMutex(queueLock_);

    if (odometryQueue_.empty() || (odometry.timestamp > odometryQueue_.back().timestamp)) {
        odometryQueue_.push_back(odometry);
    } else {
        std::cerr << "WARNING:MetricSLAMDataQueue: Ignored odometry data with time in the past.\n";
    }

    updateNextLaserData();
    dataAvailableCondition_.setPredicate(sensorInputHasArrived());
    dataAvailableCondition_.broadcast();
}


void MetricSLAMDataQueue::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    utils::AutoMutex autoMutex(queueLock_);

    if (!utils::contains(laserIds_, scan.laserId)) {
        laserIds_.push_back(scan.laserId);
    }

    auto& queue = laserQueues_[scan.laserId];

    // Only bother taking data more recent than the most recent laser data
    if ((queue.empty() || (scan.timestamp > queue.back().timestamp))
        && ((!isReceivingIMU_ || has_data_before_time(imuQueue_, scan.timestamp))
            && has_data_before_time(odometryQueue_, scan.timestamp))) {
        queue.push_back(scan);
    } else {
        std::cerr << "WARNING:MetricSLAMDataQueue: Ignoring laser data from the past.\n";
    }

    updateNextLaserData();
    dataAvailableCondition_.setPredicate(sensorInputHasArrived());
    dataAvailableCondition_.broadcast();
}


void MetricSLAMDataQueue::handleData(const motion_state_t& motion, const std::string& channel)
{
    utils::AutoMutex autoLock(queueLock_);

    velocity_ = motion.velocity;

    // Velocity information isn't required for proper functioning, so don't wake up the thread
}


bool MetricSLAMDataQueue::waitForData(void)
{
    return !dataAvailableCondition_.timedWait(100) && sensorInputHasArrived();
}


metric_slam_data_t MetricSLAMDataQueue::readData(void)
{
    utils::AutoMutex autoMutex(queueLock_);

    assert(sensorInputHasArrived());   // Must only get sensor input if it has actually arrived, otherwise inconsistent
                                       // state can result
    assert(motionQueuesAreSynced_ || !isReceivingIMU_);

    metric_slam_data_t data;

    moveQueuedDataToSensorInput(data, laserTimestamp_);

    data.startTime = previousEndTime_;
    data.endTime = laserTimestamp_;
    previousEndTime_ = data.endTime;

    if (data.startTime > data.endTime) {
        std::cout << "Laser timestamp:" << laserTimestamp_ << " Laser period:" << data.laser.scanPeriod << '\n';
        assert(data.startTime < data.endTime);
    }

    changeActiveLaserIfNeeded();
    dataAvailableCondition_.setPredicate(false);

    return data;
}


bool MetricSLAMDataQueue::sensorInputHasArrived(void)
{
    return (previousEndTime_ < laserTimestamp_) && hasMotionDataForTime(laserTimestamp_);
}


bool MetricSLAMDataQueue::hasMotionDataForTime(int64_t time) const
{
    // Ensure enough odometry data has arrived
    return (has_data_before_time(odometryQueue_, time) && has_data_after_time(odometryQueue_, time))
      // Don't require IMU data, but if it is arriving, then ensure we have enough data and the queues are synced
      && (!isReceivingIMU_
          || (motionQueuesAreSynced_ && has_data_before_time(imuQueue_, time) && has_data_after_time(imuQueue_, time)));
}


void MetricSLAMDataQueue::updateNextLaserData(void)
{
    // Once a scan has arrived after the desired period, select the laser scan that produces a time closest to the
    // desired update period
    if (laserIds_.empty()) {
        return;
    }

    auto& laserQueue = laserQueues_[laserIds_[activeLaserIndex_]];

    // If there is data in the queue and enough time has elapsed for an update to be considered, then see if new data
    // has actually arrived.
    if (!laserQueue.empty() && (laserQueue.back().timestamp - laserTimestamp_) >= targetUpdatePeriodUs_) {
        // The laser to use is the most recent one for which proper odometry data exists
        auto laserIt = std::find_if(laserQueue.rbegin(), laserQueue.rend(), [this](const polar_laser_scan_t& scan) {
            return hasMotionDataForTime(scan.timestamp);
        });

        if (laserIt != laserQueue.rend()) {
            laserData_ = *laserIt;
            laserTimestamp_ = laserData_.timestamp;

            // Erase all laser data before and including the scan that was used
            laserQueue.erase(laserQueue.begin(), laserIt.base());
        }
    }
}


void MetricSLAMDataQueue::moveQueuedDataToSensorInput(metric_slam_data_t& data, int64_t endTime)
{
    data.laser = laserData_;
    data.velocity = velocity_;

    move_sensor_data(data.imu, imuQueue_, 0, endTime);
    move_sensor_data(data.odometry, odometryQueue_, 0, endTime);

    // Interpolate an IMU measurement (if there's data)
    // Only need to interpolate if the data time is less than the end time (which is almost every single case)
    if (!imuQueue_.empty() && (data.imu.back().timestamp < endTime)) {
        auto interpolatedIMU = interpolate_imu_data(data.imu.back(), imuQueue_.front(), endTime);
        data.imu.push_back(interpolatedIMU.first);
        imuQueue_.front() =
          interpolatedIMU
            .second;   // Replace the previous IMU measurement with the measurement updated with correct time deltas
        imuQueue_.push_front(interpolatedIMU.first);

        // Every time an interpolated measurement is created, it pushes back the IMU sequence number by 1
        ++imuSequenceOffset_;
    } else if (!data.imu.empty() && (data.imu.back().timestamp == endTime)) {
        imuQueue_.push_front(data.imu.back());
    }

    // Interpolate an odometry measurement for the front of the queue if it was exactly matched with the laser time
    // which is unlikely
    if (data.odometry.back().timestamp < endTime) {
        auto interpolatedOdometry = interpolate_odometry(data.odometry.back(), odometryQueue_.front(), endTime);
        data.odometry.push_back(interpolatedOdometry);
        odometryQueue_.push_front(interpolatedOdometry);
    } else if (data.odometry.back().timestamp == endTime) {
        odometryQueue_.push_front(data.odometry.back());
    }

    data.scanLines = calculate_scan_lines(data.laser);
}


void MetricSLAMDataQueue::changeActiveLaserIfNeeded(void)
{
    activeLaserIndex_ = (activeLaserIndex_ + 1) % laserIds_.size();
}


void MetricSLAMDataQueue::synchronizeMotionDataQueues(void)
{
    // To synchronize the queues, need to find the first odometry value for which there is an IMU value before it
    // and after it. Throw away all IMU data before this time and set the front of the queue to be the IMU value
    // interpolated at the odometry timestamp.

    // There must be at least two IMU values and an odometry value to attempt (though not guarantee) the success
    // of the synchronization
    if ((imuQueue_.size() < 2) || odometryQueue_.empty()) {
        return;
    }

    // If there isn't any odometry before the IMU data, then find the first odometry value with IMU before and after it
    // and erase all odometry up to this time. Then the sync can be performed
    if (imuQueue_.front().timestamp > odometryQueue_.front().timestamp) {
        int64_t startImuTime = imuQueue_.front().timestamp;

        utils::erase_remove_if(odometryQueue_, [startImuTime](const odometry_t& odom) {
            return odom.timestamp < startImuTime;
        });
    }

    // If there wasn't any valid odometry data, then keep waiting for it to arrive
    if (odometryQueue_.empty()) {
        return;
    }

    // At this point, odom.timestamp >= imu.timestamp
    assert(odometryQueue_.front().timestamp >= imuQueue_.front().timestamp);

    int64_t syncTime = odometryQueue_.front().timestamp;

    // Find the first IMU value with a time after the synchronization time
    auto imuAfterIt = std::find_if(imuQueue_.begin(), imuQueue_.end(), [syncTime](const imu_data_t& imu) {
        return imu.timestamp > syncTime;
    });

    // imuAfterIt > begin -- need to be able to subtract 1 fromt after it to get range for the interpolation
    assert(imuAfterIt > imuQueue_.begin());
    imuQueue_.erase(imuQueue_.begin(), imuAfterIt - 1);   // erase everything up to one before the after, which will
                                                          // mean the first two values hold the IMU values on either
                                                          // side of the first odometry data time

    assert(imuQueue_.size() >= 2);

    auto interpolatedIMU = interpolate_imu_data(imuQueue_[0], imuQueue_[1], syncTime);
    imuQueue_[0] = interpolatedIMU.first;
    imuQueue_[1] = interpolatedIMU.second;
    // Every time an interpolated measurement is created, it pushes back the IMU sequence number by 1
    ++imuSequenceOffset_;

    motionQueuesAreSynced_ = true;

    std::cout << "INFO: MetricSLAMDataQueue: Synchronized IMU and odometry at " << syncTime << '\n';
}


template <typename T>
bool has_data_before_time(const std::deque<T>& data, int64_t timestamp)
{
    return !data.empty() && (data.front().timestamp < timestamp);
}


template <typename T>
bool has_data_after_time(const std::deque<T>& data, int64_t timestamp)
{
    return !data.empty() && (data.back().timestamp >= timestamp);
}


template <typename T>
std::pair<std::size_t, std::size_t> indices_between(const std::deque<T>& data, int64_t startTime, int64_t endTime)
{
    std::pair<std::size_t, std::size_t> indices(0, 0);

    for (std::size_t n = 0; n < data.size(); ++n) {
        if (data[n].timestamp <= startTime) {
            indices.first = n;
        } else if (data[n].timestamp > endTime) {
            indices.second = n;
            break;
        }
    }

    if (indices.second == 0) {
        indices.second = data.size();
    }

    return indices;
}


template <typename T>
void move_sensor_data(std::vector<T>& to, std::deque<T>& from, int64_t startTime, int64_t endTime)
{
    std::pair<std::size_t, std::size_t> range = indices_between(from, startTime, endTime);

    to.clear();

    if (!from.empty()) {
        to.insert(to.begin(), from.begin() + range.first, from.begin() + range.second);

        from.erase(from.begin(), from.begin() + range.second);
    }
}


laser::laser_scan_lines_t calculate_scan_lines(const polar_laser_scan_t& scan)
{
    // Hard-code in the parameters that have been used for years for calculating the scan lines
    laser::quick_split_params_t params;
    params.clusterDistance = 0.5;
    params.maxDistanceFromLine = 0.05;
    params.minPoints = 8;

    cartesian_laser_scan_t cartesian;
    polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesian, true);

    auto extracted = laser::quick_split(scan, cartesian, params);

    laser::laser_scan_lines_t lines;
    lines.lines = extracted.lines;
    lines.scanPointToLineIndices = extracted.indices;
    lines.scan = scan;
    return lines;
}

}   // namespace hssh
}   // namespace vulcan
