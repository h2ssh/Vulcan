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
* Declaration of LocalTopologyDirector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_DIRECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_DIRECTOR_H

#include "system/director.h"

#include "utils/mutex.h"
#include "utils/condition_variable.h"

#include "core/pose.h"
#include "laser/laser_scan_lines.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"

#include "hssh/local_topological/cmd_line.h"
#include "hssh/local_topological/params.h"
#include "hssh/local_topological/mode.h"
#include "hssh/local_topological/area_detector.h"
#include "hssh/local_topological/event_detector.h"
#include "hssh/local_topological/localizer.h"

#include "hssh/utils/lpm_axes_calculator.h"
#include "utils/locked_bool.h"
#include "utils/locked_double_buffer.h"

#include <atomic>
#include <vector>

namespace vulcan
{
namespace hssh
{

class LocalTopoCommand;
class SmallScaleSpaceBoundary;

/**
* LocalTopologyDirector manages the data processing for the local topology module. The
* local topology module needs an LPM and a pose to determine the local topology
* of the current LPM.
*
* The calculation flow is:
*
*   - Build the VoronoiSkeletonGrid from the current LPM
*   - Find the gateways
*   - Filter the spurious gateways
*   - Create places for the decision points
*
* The behavior of local_topo_hssh can be controlled with two command-line arguments to support diagnostics:
*
*   - save-events : save all LocalAreaEvents generated while the module is running
*   - save-map    : save the LocalTopoMap that exists when the module is exited
*
* Saving events is intended to be used for evaluating the quality of the place detection. Saving the events ensures
* that no events are missed due to LCM traffic. The command-line approach allows a script to be used more easily.
*
* Saving the map is inteded to be used for evaluating the quality of the place classification. Saving the map on close
* guarantees the map created by labeling a global metrical map is actually saved and not lost in transmission, as too
* often happens.
*/
class LocalTopologyDirector : public system::Director
{
public:

    /**
    * Constructor for LocalTopologyDirector.
    *
    * \param    commandLine         Command-line arguments passed to the module
    * \param    config              Config file to use for loading the module
    */
    LocalTopologyDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    /**
    * Destructor for LocalTopologyDirector.
    */
    virtual ~LocalTopologyDirector(void);

    LocalTopoMap oneShotLabeling(const LocalPerceptualMap& lpm);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const LocalPerceptualMap& grid, const std::string& channel);
    void handleData(const LocalPose&          pose, const std::string& channel);
    void handleData(const std::shared_ptr<LocalTopoCommand>& command, const std::string& channel);


private:

    // Data processing classes
    AreaDetector  areaDetector_;
    EventDetector eventDetector_;
    LocalTopoLocalizer localizer_;
    std::unique_ptr<SmallScaleSpaceBoundary> boundary_;

    std::atomic<LocalTopoMode> mode_;        ///< Current mode of operation

    LocalPose searchPose_;
    LocalTopoMap map_;
    LocalLocation location_;
    LocalAreaEventVec events_;

    // Data used for processing
    utils::LockedDoubleBuffer<LocalPerceptualMap> lpm;
    utils::LockedDoubleBuffer<LocalPose>          currentPose;

    int referenceFrameIndex;
    int failedLpmIndex_;

    bool haveNewLpm_;
    bool haveInitializedPose_;

    bool shouldSaveEvents_;
    LocalAreaEventVec allEvents_;
    std::string eventsFilename_;

    bool shouldSaveMap_;
    std::string mapFilename_;
    int64_t lastMapSentTimeUs_ = 0;

    utils::Mutex eventDetectionLock_;       // adding poses to event detection happens on one thread, detecting on the other
    utils::ConditionVariable inputTrigger_;

    // Check whether the conditions have been met for doing an update for the local topology calculation
    bool shouldProcessData(void);

    void findAreas (system::ModuleCommunicator& communicator);
    void findEvents(system::ModuleCommunicator& communicator);
    void maintainSmallScaleSpace(system::ModuleCommunicator& communicator);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_DIRECTOR_H
