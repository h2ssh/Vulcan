/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.cpp
* \author   Collin Johnson
*
* Definition of ObjectTrackerDirector.
*/

#include <tracker/director.h>
#include <tracker/goal_predictor.h>
#include <tracker/object_detector.h>
#include <tracker/object_tracker.h>
#include <tracker/tracking_environment.h>
#include <hssh/local_topological/areas/serialization.h>
#include <laser/moving_laser_scan.h>
#include <core/motion_state.h>
#include <system/module_communicator.h>
#include <utils/algorithm_ext.h>
#include <utils/auto_mutex.h>
#include <cassert>

namespace vulcan
{
namespace tracker
{

ObjectTrackerDirector::ObjectTrackerDirector(int32_t outputPeriodMs,
                                             std::unique_ptr<ObjectDetector> detector,
                                             std::unique_ptr<ObjectTracker>  tracker,
                                             std::unique_ptr<GoalPredictor>  predictor,
                                             const hssh::LocalPerceptualMap* lpm,
                                             const hssh::LocalTopoMap*       ltm)
: detector_(std::move(detector))
, tracker_(std::move(tracker))
, predictor_(std::move(predictor))
, poses_(0.0f, 500)   // store all poses, keep around 10 seconds' worth of poses
, currentLTM_(ltm)
, outputPeriodUs_(outputPeriodMs * 1000ll)
, dataTrigger_(false)
{
    if (lpm)
    {
        handleData(*lpm, "constructor");
    }
}


ObjectTrackerDirector::~ObjectTrackerDirector(void)
{
    // For std::unique_ptr
}


void ObjectTrackerDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<polar_laser_scan_t>(this);
    communicator.subscribeTo<hssh::LocalPerceptualMap>(this);
    communicator.subscribeTo<hssh::LocalTopoMap>(this);
    communicator.subscribeTo<motion_state_t>(this);
}


system::TriggerStatus ObjectTrackerDirector::waitForTrigger(void)
{
    return !dataTrigger_.timedWait(100) ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus ObjectTrackerDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    copySharedToLocalState();
    trackObjectsInCurrentScan(communicator);

    dataTrigger_.setPredicate(haveDataForUpdate());

    // The object_tracker is always running
    return system::UpdateStatus::running;
}


void ObjectTrackerDirector::shutdown(system::ModuleCommunicator& transmitter)
{
    // No specific shutdown needed
}


void ObjectTrackerDirector::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    lpmBuffer_.write(lpm);
}


void ObjectTrackerDirector::handleData(const hssh::LocalTopoMap& ltm, const std::string& channel)
{
    ltmBuffer_.write(ltm);
}


void ObjectTrackerDirector::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    {
        utils::AutoMutex autoLock(dataLock_);
        scans_.push_back(scan);

        laserIds_.insert(scan.laserId);
        haveMultipleLasers_ = laserIds_.size() > 1;
    }

    if(haveDataForUpdate())
    {
        dataTrigger_.setPredicate(true);
        dataTrigger_.broadcast();
    }
}


void ObjectTrackerDirector::handleData(const motion_state_t& state, const std::string& channel)
{
    {
        utils::AutoMutex autoLock(dataLock_);
        auto poseToAdd = state.pose;
        poseToAdd.timestamp = state.timestamp;
        poses_.addPose(poseToAdd);
        robotPose_ = state.poseDistribution;
    }

    if(haveDataForUpdate())
    {
        dataTrigger_.setPredicate(true);
        dataTrigger_.broadcast();
    }
}


bool ObjectTrackerDirector::haveDataForUpdate(void)
{
    utils::AutoMutex autoLock(dataLock_);

    // No data if:
    // No scans available
    if(scans_.empty() || (currentLPM_ == nullptr && !lpmBuffer_.hasData()))
    {
        return false;
    }

    // Or a pose doesn't exist before and after the first scan's timestamp, meaning that proper interpolation of the
    // laser position can't be performed
    std::vector<int> currentIds;
    // If multiple lasers are available, require switching between lasers to keep from starving one of the trackers
    for(auto& scan : scans_)
    {
        if(poses_.hasPoseAfterTime(scan.timestamp + utils::sec_to_usec(scan.scanPeriod))
            && !utils::contains(currentIds, scan.laserId))
        {
            currentIds.push_back(scan.laserId);
        }
    }

    return currentIds.size() == laserIds_.size();
}


void ObjectTrackerDirector::copySharedToLocalState(void)
{
    assert(lpmBuffer_.hasData() || currentLPM_);

    if(lpmBuffer_.hasData())
    {
        lpmBuffer_.swapBuffers();
        currentLPM_ = &lpmBuffer_.read();
    }

    if(ltmBuffer_.hasData())
    {
        ltmBuffer_.swapBuffers();
        currentLTM_ = &ltmBuffer_.read();
    }

    utils::AutoMutex autoLock(dataLock_);

    // Sort scans in order of increasing timestamp
    std::sort(scans_.begin(), scans_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp < rhs.timestamp;
    });

    // The next scan to use will have some pose data after it, allowing proper interpolation of the laser values and a
    // better position estimate for where the object is
    // Find the beyond-the-end of the scans with pose data after their own time
//     auto scanIt = std::find_if(scans_.rbegin(), scans_.rend(), [this](const polar_laser_scan_t& scan) {
//         return poses_.hasPoseAfterTime(scan.timestamp + utils::sec_to_usec(scan.scanPeriod))
//             && (!haveMultipleLasers_ || (scan.laserId != lastLaserId_));
//     });

    currentScans_.clear();
    std::size_t lastUsedIndex = 0;
    for(std::size_t n = scans_.size(); n > 0; --n)
    {
        auto& currScan = scans_[n - 1];

        bool haveScanWithId = utils::contains_if(currentScans_, [&currScan](const auto& movingScan) {
            return currScan.laserId == movingScan.laserId();
        });

        // If this timestamp is newer than the previous update and sufficient data, create a moving laser scan for it
        if(!haveScanWithId
            && (currScan.timestamp >= lastUpdateTime_)
            && poses_.hasPoseAfterTime(currScan.timestamp + utils::sec_to_usec(currScan.scanPeriod)))
        {
            auto startPose = poses_.poseAt(currScan.timestamp);
            auto endPose = poses_.poseAt(currScan.timestamp + utils::sec_to_usec(currScan.scanPeriod));
            currentScans_.emplace_back(currScan, startPose, endPose);
            lastUsedIndex = n;
        }
    }

    currentRobotPose_ = robotPose_;

    if(!currentScans_.empty())
    {
        lastUpdateTime_ = currentScans_.back().timestamp();
        lastLaserId_ = currentScans_.back().laserId();
    }

    // Erase all scan data that was used for this update
    scans_.erase(scans_.begin(), scans_.begin() + lastUsedIndex);
    poses_.clearBefore(lastUpdateTime_);
}


void ObjectTrackerDirector::trackObjectsInCurrentScan(system::ModuleCommunicator& communicator)
{
    // Only can track if there are scans available
    if(!currentScans_.empty())
    {
        tracking_environment_t env = { lastUpdateTime_, currentLPM_, currentLTM_ };
        LaserObjectCollection finalLaserObjects;    // store the final processing of the laser objects

        auto laserObjects = detector_->detectObjects(currentScans_, currentRobotPose_, *currentLPM_);
        auto trackedObjects = tracker_->trackObjects(laserObjects, &finalLaserObjects);
        auto dynamicObjects = predictor_->predictGoals(trackedObjects, env);

        communicator.sendMessage(finalLaserObjects);

        if(lastUpdateTime_ - lastTransmitTime_ > outputPeriodUs_)
        {
            std::cout << "INFO: ObjectTracker: Num tracked objects: " << dynamicObjects.size() 
                << " Duration (ms): " << ((lastUpdateTime_ - lastTransmitTime_) / 1000) << '\n';
            communicator.sendMessage(dynamicObjects);
            lastTransmitTime_ = lastUpdateTime_;
        }
    }
}

} // namespace tracker
} // namespace vulcan
