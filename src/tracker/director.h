/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     director.h
 * \author   Collin Johnson
 *
 * Declaration of ObjectTrackerDirector.
 */

#ifndef TRACKER_DIRECTOR_H
#define TRACKER_DIRECTOR_H

#include "core/laser_scan.h"
#include "core/pose_distribution.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_topological/local_topo_map.h"
#include "laser/moving_laser_scan.h"
#include "system/director.h"
#include "utils/condition_variable.h"
#include "utils/locked_double_buffer.h"
#include "utils/pose_trace.h"
#include <unordered_set>
#include <vector>

namespace vulcan
{
struct motion_state_t;

namespace tracker
{

class GoalPredictor;
class ObjectDetector;
class ObjectTracker;

/**
 * ObjectTrackerDirector
 */
class ObjectTrackerDirector : public system::Director
{
public:
    /**
     * Constructor for ObjectTrackerDirector.
     *
     * \param    outputPeriodMs          Period for output of the DynamicObjectCollection
     * \param    detector                Detector to use for segmenting a laser scan into objects
     * \param    tracker                 Tracker to use for tracking objects over time
     * \param    predictor               GoalPredictor instance to use for dynamic object goal estimation
     * \param    ltm                     LocalTopoMap to use for goal estimation -- might be null, in which case
     *                                   the map received from local_topo_hssh will be used for estimation
     */
    ObjectTrackerDirector(int32_t outputPeriodMs,
                          std::unique_ptr<ObjectDetector> detector,
                          std::unique_ptr<ObjectTracker> tracker,
                          std::unique_ptr<GoalPredictor> predictor,
                          const hssh::LocalPerceptualMap* lpm,
                          const hssh::LocalTopoMap* ltm);

    /**
     * Destructor for ObjectTrackerDirector.
     */
    ~ObjectTrackerDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void handleData(const hssh::LocalTopoMap& ltm, const std::string& channel);
    void handleData(const polar_laser_scan_t& scan, const std::string& channel);
    void handleData(const motion_state_t& state, const std::string& channel);

private:
    std::unique_ptr<ObjectDetector> detector_;
    std::unique_ptr<ObjectTracker> tracker_;
    std::unique_ptr<GoalPredictor> predictor_;
    utils::PoseTrace poses_;
    pose_distribution_t robotPose_;

    // Shared state where the input thread and computation thread will talk
    utils::LockedDoubleBuffer<hssh::LocalPerceptualMap> lpmBuffer_;
    utils::LockedDoubleBuffer<hssh::LocalTopoMap> ltmBuffer_;
    std::deque<polar_laser_scan_t> scans_;
    std::unordered_set<int> laserIds_;
    int lastLaserId_ = -1;
    bool haveMultipleLasers_ = false;

    // Local state used only by the tracker
    const hssh::LocalPerceptualMap* currentLPM_ = nullptr;
    const hssh::LocalTopoMap* currentLTM_ = nullptr;
    std::vector<laser::MovingLaserScan> currentScans_;
    polar_laser_scan_t currentScan_;
    pose_distribution_t currentRobotPose_;

    int64_t lastUpdateTime_ = 0;
    int64_t outputPeriodUs_;
    int64_t lastTransmitTime_ = 0;

    utils::ConditionVariable dataTrigger_;
    utils::Mutex dataLock_;

    bool haveDataForUpdate(void);        // acquires dataLock_
    void copySharedToLocalState(void);   // acquires dataLock_
    void trackObjectsInCurrentScan(system::ModuleCommunicator& communicator);
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_DIRECTOR_H
