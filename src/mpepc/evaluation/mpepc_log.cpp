/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mpepc_log.cpp
* \author   Jong Jin Park
*
* Definition of MPEPCLog.
*/

#include <mpepc/evaluation/mpepc_log.h>
#include <tracker/objects/serialization.h>
#include <system/module_communicator.h>
#include <utils/algorithm_ext.h>
#include <algorithm>
#include <limits>
#include <sstream>

namespace vulcan
{
namespace mpepc
{


// Utility functions for finding timestamped data
template <class Iter>
Iter beginDuration(Iter begin, Iter end, int64_t time, int64_t start);

template <class Iter>
Iter endDuration(Iter begin, Iter end, int64_t time, int64_t start);


//////////////////////////////////////// MPEPCLog implementation ////////////////////////////////////////////
MPEPCLog::MPEPCLog(void)
{
    // Nothing needed for default construction
}


MPEPCLog::MPEPCLog(const std::string& logFilename)
: name_(logFilename)
, startTime_(std::numeric_limits<int64_t>::max())
{
    // Load all values from the provided LCM log. The LCM callbacks instance that will interact with LCM and callback
    // to add the values to the sensor log.

    // To create a log provider, we can use a special Url for LCM. The Url to use was found in the source for the
    // lcm-logplayer. The interesting command is setting speed=0, which causes the log to playback as fast as it can
    // be read, as opposed to waiting between messages. Info on speed=0 can be found in lcm/lcm_file.c.
    std::ostringstream logProviderUrl;
    logProviderUrl << "file://" << logFilename << "?speed=0"; // default mode is read mode, but have to read LCM code carefully to actually know that
    // provide url for both system and debug messages.
    comm_ = std::make_unique<system::ModuleCommunicator>(logProviderUrl.str(), logProviderUrl.str());

    // Subscribe to each message in the log
    comm_->subscribeTo<mpepc::trajectory_planner_debug_info_t>(this);
    comm_->subscribeTo<motion_state_t>(this);
    comm_->subscribeTo<robot::commanded_joystick_t>(this);
    comm_->subscribeTo<imu_data_t>(this);
    comm_->subscribeTo<tracker::DynamicObjectCollection>(this);

    // Process just the first message to grab some timing information
    bool haveMessage = true;
    while((duration_ == 0) && haveMessage)
    {
        haveMessage = comm_->processIncoming() == 0;
    }

    std::cout << "INFO: MPEPCLog: Loading the data from " << name_ << ". Base time:" << startTime_ << '\n';
    setTimeBase(startTime_);
}


int MPEPCLog::loadTimeRange(int64_t begin, int64_t end)
{
    // Keep reading messages until the processIncoming doesn't return success, indicating that there's no data left
    bool haveMessage = true;
    int numProcessed = 0;
    while((end == -1 || endTime_ < end) && haveMessage)
    {
        haveMessage = comm_->processIncoming() == 0;

        if(haveMessage)
        {
            ++numProcessed;
        }
    }

    // Once all data has been read, then sort all data by timestamp to ensure correct ordering
    std::sort(mpepcInfo_.begin(), mpepcInfo_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.planReleaseTimeUs < rhs.planReleaseTimeUs;
    });
    std::sort(motionState_.begin(), motionState_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp < rhs.timestamp;
    });
    std::sort(joystickCommand_.begin(), joystickCommand_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp < rhs.timestamp;
    });
    std::sort(imuData_.begin(), imuData_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp < rhs.timestamp;
    });
    std::sort(objects_.begin(), objects_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp() < rhs.timestamp();
    });

    // Erase all messages coming before the new start time
    utils::erase_remove_if(mpepcInfo_, [begin](auto& info) {
        return info.planReleaseTimeUs < begin;
    });

    utils::erase_remove_if(motionState_, [begin](auto& state) {
        return state.timestamp < begin;
    });

    utils::erase_remove_if(joystickCommand_, [begin](auto& cmd) {
        return cmd.timestamp < begin;
    });

    utils::erase_remove_if(imuData_, [begin](auto& imu) {
        return imu.timestamp < begin;
    });

    utils::erase_remove_if(objects_, [begin](auto& obj) {
        return obj.timestamp() < begin;
    });

    return numProcessed;
}


void MPEPCLog::loadAll(void)
{
    loadTimeRange(0, -1);
}


MPEPCLog::debug_info_iterator MPEPCLog::beginMPEPCInfo(int64_t timeUs) const
{
    // Find the first instance where this time is larger than the desired duration
    return std::find_if(mpepcInfo_.begin(), mpepcInfo_.end(), [timeUs, this](const auto& info) {
        return (info.planReleaseTimeUs - startTime_) >= timeUs;
    });
}


MPEPCLog::debug_info_iterator MPEPCLog::endMPEPCInfo(int64_t timeUs) const
{
    // In this case, make it strictly larger than the duration so iteration will include the closed range if it is
    // available
    return std::find_if(mpepcInfo_.begin(), mpepcInfo_.end(), [timeUs, this](const auto& info) {
        return (info.planReleaseTimeUs - startTime_) > timeUs;
    });
}


MPEPCLog::motion_state_iterator MPEPCLog::beginMotionState(int64_t timeUs) const
{
    return beginDuration(motionState_.begin(), motionState_.end(), timeUs, startTime_);
}


MPEPCLog::motion_state_iterator MPEPCLog::endMotionState(int64_t timeUs) const
{
    return endDuration(motionState_.begin(), motionState_.end(), timeUs, startTime_);
}



MPEPCLog::joystick_iterator MPEPCLog::beginCommandedJoystick(int64_t timeUs) const
{
    return beginDuration(joystickCommand_.begin(), joystickCommand_.end(), timeUs, startTime_);
}


MPEPCLog::joystick_iterator MPEPCLog::endCommandedJoystick(int64_t timeUs) const
{
    return endDuration(joystickCommand_.begin(), joystickCommand_.end(), timeUs, startTime_);
}


MPEPCLog::imu_iterator MPEPCLog::beginImu(int64_t timeUs) const
{
    return beginDuration(imuData_.begin(), imuData_.end(), timeUs, startTime_);
}


MPEPCLog::imu_iterator MPEPCLog::endImu(int64_t timeUs) const
{
    return endDuration(imuData_.begin(), imuData_.end(), timeUs, startTime_);
}


MPEPCLog::object_iterator MPEPCLog::beginObjects(int64_t timeUs) const
{
    // Find the first instance where this time is greater than or equal to the desired relative begin time
    return std::find_if(beginObjects(), endObjects(), [&](const auto& data) {
        return (data.timestamp() - startTime_) >= timeUs;
    });
}


MPEPCLog::object_iterator MPEPCLog::endObjects(int64_t timeUs) const
{
    // Find the first instance where the time is strictly greater than the relative end time
    return std::find_if(beginObjects(), endObjects(), [&](const auto& data) {
        return (data.timestamp() - startTime_) > timeUs;
    });
}


void MPEPCLog::addMPEPCInfo(const mpepc::trajectory_planner_debug_info_t& debugInfo)
{
    mpepc::trajectory_planner_debug_info_t lessDebugInfo = debugInfo;

    // avoid storing too much info
    lessDebugInfo.trajectories.clear();
    lessDebugInfo.trajectoryToInitialState.clear();
    lessDebugInfo.coarseOptimizerOutput = mpepc::robot_trajectory_debug_info_t();
    lessDebugInfo.localOptimizerOutput  = mpepc::robot_trajectory_debug_info_t();

    mpepcInfo_.push_back(lessDebugInfo);
    mpepcInfo_.back().updateEndTimeUs -= baseTime_;
    mpepcInfo_.back().updateStartTimeUs -= baseTime_;
    mpepcInfo_.back().planReleaseTimeUs -= baseTime_;
//     mpepcInfo_.push_back(debugInfo);
    updateTime(mpepcInfo_.back().planReleaseTimeUs);
}


void MPEPCLog::addMotionState(const motion_state_t& motionState)
{
    motionState_.push_back(motionState);
    motionState_.back().timestamp -= baseTime_;
    updateTime(motionState_.back().timestamp);
}


void MPEPCLog::addCommandedJoystick(const robot::commanded_joystick_t& joystick)
{
    joystickCommand_.push_back(joystick);
    joystickCommand_.back().timestamp -= baseTime_;
    updateTime(joystickCommand_.back().timestamp);
}


void MPEPCLog::addImu(const imu_data_t& imu)
{
    imuData_.push_back(imu);
    imuData_.back().timestamp -= baseTime_;
    updateTime(imuData_.back().timestamp);
}


void MPEPCLog::addDynObjCollection(const tracker::DynamicObjectCollection& objects)
{
    objects_.push_back(objects);
    objects_.back().setTimestamp(objects_.back().timestamp() - baseTime_);
    updateTime(objects_.back().timestamp());
}


void MPEPCLog::handleData(const mpepc::trajectory_planner_debug_info_t& info, const std::string& channel)
{
    addMPEPCInfo(info);
}

void MPEPCLog::handleData(const motion_state_t& state, const std::string& channel)
{
    addMotionState(state);
}

void MPEPCLog::handleData(const robot::commanded_joystick_t& joystick, const std::string& channel)
{
    addCommandedJoystick(joystick);
}

void MPEPCLog::handleData(const imu_data_t& imu, const std::string& channel)
{
    addImu(imu);
}


void MPEPCLog::handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
    addDynObjCollection(objects);
}


void MPEPCLog::updateTime(int64_t dataTimeUs)
{
    startTime_ = (startTime_ == 0) ? dataTimeUs : std::min(dataTimeUs, startTime_);
    endTime_ = std::max(dataTimeUs, endTime_);
    duration_ = std::max(dataTimeUs - startTime_, duration_);
}


void MPEPCLog::setTimeBase(int64_t timebase)
{
    baseTime_ = timebase;
    startTime_ -= timebase;
    endTime_ -= timebase;

    for(auto& m : mpepcInfo_)
    {
        m.updateEndTimeUs -= timebase;
        m.updateStartTimeUs -= timebase;
        m.planReleaseTimeUs -= timebase;
    }

    for(auto& s : motionState_)
    {
        s.timestamp -= timebase;
    }

    for(auto& c : joystickCommand_)
    {
        c.timestamp -= timebase;
    }

    for(auto& i : imuData_)
    {
        i.timestamp -= timebase;
    }

    for(auto& objs : objects_)
    {
        objs.setTimestamp(objs.timestamp() - timebase);
    }
}


template <class Iter>
Iter beginDuration(Iter begin, Iter end, int64_t time, int64_t start)
{
    // Find the first instance where this time is greater than or equal to the desired relative begin time
    return std::find_if(begin, end, [&](const auto& data) {
        return (data.timestamp - start) >= time;
    });
}


template <class Iter>
Iter endDuration(Iter begin, Iter end, int64_t time, int64_t start)
{
    // Find the first instance where the time is strictly greater than the relative end time
    return std::find_if(begin, end, [&](const auto& data) {
        return (data.timestamp - start) > time;
    });
}

} // namespace mpepc
} // namespace vulcan
