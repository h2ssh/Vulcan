/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mpepc_log.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of MPEPCLog.
*/

#ifndef MPEPC_EVALUATION_MPEPC_LOG_H
#define MPEPC_EVALUATION_MPEPC_LOG_H

#include <mpepc/trajectory/trajectory_planner_info.h>
#include <core/imu_data.h>
#include <system/module_communicator.h>
#include <tracker/dynamic_object_collection.h>
#include <memory>

namespace vulcan
{
namespace mpepc
{

/**
* MPEPCLog is a utility for loading the contents of a driving data log into RAM. MPEPCLog exists to avoid needing to use
* a logplayer to read all the driving data from a log for processing, be it for doing calculations or converting to a
* Matlab-friendly format.
*
* MPEPCLog is based on SensorLog by Collin Johnson. This version listens to both system and debug messages.
*
* Accessing iterators for the data sources all provide two forms. One form provides iterators over the entire sequence.
* The other form allows for iteration over specific durations of time relative to the start time of the log. This
* approach provides easy access to a subset of the data of interest.
*/
class MPEPCLog
{
public:

    using debug_info_iterator = std::vector<mpepc::trajectory_planner_debug_info_t>::const_iterator;
    using motion_state_iterator = std::vector<motion_state_t>::const_iterator;
    using joystick_iterator = std::vector<robot::commanded_joystick_t>::const_iterator;
    using imu_iterator = std::vector<imu_data_t>::const_iterator;
    using object_iterator = std::vector<tracker::DynamicObjectCollection>::const_iterator;

    /**
    * Default constructor for MPEPCLog.
    *
    * Create an empty log.
    */
    MPEPCLog(void);

    /**
    * Constructor for MPEPCLog.
    *
    * Create a MPEPCLog containing all data in the provided LCM log.
    *
    * \param    logFilename         Name of the log to be loaded
    */
    MPEPCLog(const std::string& logFilename);

    // Disable copying, because these logs can be huge. Only moves are allowed for sanity's sake
    MPEPCLog(const MPEPCLog&) = delete;
    MPEPCLog& operator=(const MPEPCLog&) = delete;

    MPEPCLog(MPEPCLog&&) = default;
    MPEPCLog& operator=(MPEPCLog&&) = default;

    /**
    * name retrieves the name of the sensor log. It is just the end of the filename, not the full path.
    */
    std::string name(void) const { return name_; }

    /**
    * loadTimeRange loads all data in the specified time range. This range will toss out currently loaded data that
    * isn't in the range and then append all data to make it in the range.
    *
    * A consequence here is that you need to process the log in increasing order.
    *
    * \pre  begin >= startTimeUs
    * \param    begin           Start time of the range
    * \param    end             End time of the range
    * \return   Number of new messages read.
    */
    int loadTimeRange(int64_t begin, int64_t end);

    /**
    * loadAll loads the entire log.
    *
    * WARNING: A log might actually be too big to hold in RAM, so this call could fail!
    */
    void loadAll(void);

    /**
    * startTimeUs retrieves the start time of the log in microseconds from an arbitrary time in the past.
    */
    int64_t startTimeUs(void) const { return startTime_; }

    /**
    * durationUs retrieves the duration of the full log in microseconds.
    */
    int64_t durationUs(void) const { return duration_; }

    // Iterator access for the data in the log
    std::size_t sizeMPEPCInfo(void) const { return mpepcInfo_.size(); }
    debug_info_iterator beginMPEPCInfo(void) const { return mpepcInfo_.begin(); }
    debug_info_iterator endMPEPCInfo(void) const { return mpepcInfo_.end(); }
    debug_info_iterator beginMPEPCInfo(int64_t timeUs) const;
    debug_info_iterator endMPEPCInfo(int64_t timeUs) const;

    std::size_t sizeMotionState(void) const { return motionState_.size(); }
    motion_state_iterator beginMotionState(void) const { return motionState_.begin(); }
    motion_state_iterator endMotionState(void) const { return motionState_.end(); }
    motion_state_iterator beginMotionState(int64_t timeUs) const;
    motion_state_iterator endMotionState(int64_t timeUs) const;


    std::size_t sizeCommandedJoystick(void) const { return joystickCommand_.size(); }
    joystick_iterator beginCommandedJoystick(void) const { return joystickCommand_.begin(); }
    joystick_iterator endCommandedJoystick(void) const { return joystickCommand_.end(); }
    joystick_iterator beginCommandedJoystick(int64_t timeUs) const;
    joystick_iterator endCommandedJoystick(int64_t timeUs) const;

    std::size_t sizeImu(void) const { return imuData_.size(); }
    imu_iterator beginImu(void) const { return imuData_.begin(); }
    imu_iterator endImu(void) const { return imuData_.end(); }
    imu_iterator beginImu(int64_t timeUs) const;
    imu_iterator endImu(int64_t timeUs) const;

    std::size_t sizeObjects(void) const { return objects_.size(); }
    object_iterator beginObjects(void) const { return objects_.begin(); }
    object_iterator endObjects(void) const { return objects_.end(); }
    object_iterator beginObjects(int64_t timeUs) const;
    object_iterator endObjects(int64_t timeUs) const;

    // Add the various type of data to the log
    void addMPEPCInfo(const mpepc::trajectory_planner_debug_info_t& debugInfo);
    void addMotionState(const motion_state_t& motionState);
    void addCommandedJoystick(const robot::commanded_joystick_t& joystick);
    void addImu(const imu_data_t& imu);
    void addDynObjCollection(const tracker::DynamicObjectCollection& objects);

    // Handlers for interacting with LCM
    void handleData(const mpepc::trajectory_planner_debug_info_t& info, const std::string& channel);
    void handleData(const motion_state_t& state, const std::string& channel);
    void handleData(const robot::commanded_joystick_t& joystick, const std::string& channel);
    void handleData(const imu_data_t& imu, const std::string& channel);
    void handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel);

private:

    std::string name_;
    int64_t startTime_ = 0;
    int64_t endTime_ = 0;
    int64_t duration_ = 0;
    int64_t baseTime_ = 0;

    std::vector<mpepc::trajectory_planner_debug_info_t> mpepcInfo_;
    std::vector<motion_state_t> motionState_;
    std::vector<robot::commanded_joystick_t> joystickCommand_;
    std::vector<imu_data_t> imuData_;
    std::vector<tracker::DynamicObjectCollection> objects_;

    std::unique_ptr<system::ModuleCommunicator> comm_;

    void updateTime(int64_t dataTimeUs);
    void setTimeBase(int64_t timebase);
};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_EVALUATION_MPEPC_LOG_H
