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
* Declaration of LocalMetricDirector.
*/

#ifndef HSSH_LOCAL_METRIC_DIRECTOR_H
#define HSSH_LOCAL_METRIC_DIRECTOR_H

#include <system/director.h>
#include <laser/laser_line_extractor.h>
#include <hssh/local_metric/params.h>
#include <hssh/local_metric/pose.h>
#include <hssh/local_metric/messages.h>
#include <hssh/metrical/relocalization/types.h>
#include <hssh/local_metric/debug_info.h>
#include <hssh/metrical/mapping/mapper.h>
#include <hssh/metrical/relocalization/metric_relocalizer.h>
#include <hssh/metrical/relocalization/debug_info.h>
#include <hssh/local_metric/multifloor/mapper.h>
#include <hssh/metrical/data_queue.h>
#include <lcmtypes/commands/load_lpm_command.h>
#include <lcmtypes/commands/save_lpm_command.h>
#include <core/motion_state.h>
#include <core/pose_distribution.h>
#include <utils/mutex.h>
#include <utils/locked_double_buffer.h>
#include <utils/locked_queue.h>
#include <utils/pose_trace.h>

namespace vulcan
{
namespace hssh
{

class  LocalMetricCommand;
class  Localizer;
struct local_metric_params_t;

const std::string kSavePosesArg("save-poses");

/**
* LocalMetricDirector organizes the computation for the LPM. The Director receives data from some
* producer of sensor input, process that data, and provides the output to a consumer
* of the computed output.
*
* * The local_metric_hssh modules accepts one optional command-line argument controlling the behavior of the module:
*
*   - save-poses  : save the estimated poses to a PoseTrace that will be saved to disk on exit (provide the filename)
*/
class LocalMetricDirector : public system::Director
{
public:

    /**
    * Constructor for LocalMetricDirector.
    *
    * \param    commandLine         Command-line arguments from startup
    * \param    config              Config file to be used for configuring the module
    */
    LocalMetricDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    virtual ~LocalMetricDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handler interface
    void handleData(const robot::elevator_t&                   elevator, const std::string& channel);
    void handleData(const std::shared_ptr<LocalMetricCommand>& command,  const std::string& channel);
    void handleData(const motion_state_t&               state,    const std::string& channel);

    // Messages to handle for the external shim layer for using pure LCM messages to talk with this module
    void handleData(const vulcan_lcm::save_lpm_command& command, const std::string& channel);
    void handleData(const vulcan_lcm::load_lpm_command& command, const std::string& channel);

private:

    local_metric_hssh_params_t params;

    LocalMetricMode        mode;
    MetricSLAMDataQueue    inputQueue;
    std::unique_ptr<Localizer> localizer;
    Mapper                 mapper;
    Mapper                 highResMapper;
    MetricRelocalizer      relocalizer;
    MultiFloorMapper       multiFloor;

    utils::LockedDoubleBuffer<motion_state_t> motionState;
    std::deque<robot::elevator_t>                    elevatorQueue;
    utils::Mutex                                     elevatorLock;

    utils::LockedQueue<std::shared_ptr<LocalMetricCommand>> commands;

    pose_t              priorPose;
    pose_t              currentPose;
    pose_distribution_t currentPoseDistribution;
    velocity_t          currentVelocity;

    local_metric_localization_debug_info_t   debugInfo;
    local_metric_relocalization_debug_info_t relocalizationInfo;

    int     updateCount;
    int32_t lastReferenceIndex;
    int64_t previousMapTransmissionTime;
    bool    sentInitialMap;
    bool    sentLPM;
    bool    haveRelocalizationInfo;
    bool    haveRelocalizationMessage;

    int64_t totalLocalizationTime;
    int     numLocalizations;

//     utils::PoseTrace poseTrace_;
    std::vector<LocalPose> poseTrace_;
    bool shouldSavePoses_;
    std::string poseTraceName_;

    const int64_t mapTransmissionPeriod;
    const bool    shouldSendGlassMap;

    std::set<int> initializedLasers;

    // system::Director interface
    void processAvailableData(void);
    void transmitCalculatedOutput(system::ModuleCommunicator& communicator);

    void processMessages(const metric_slam_data_t& data);

    void processLPM          (const metric_slam_data_t& data);
    void initializeLPM       (const metric_slam_data_t& data);
    void updateLPM           (const metric_slam_data_t& data);
    void updateLocalization  (const metric_slam_data_t& data);
    void updateMap           (const metric_slam_data_t& data);
    void updateHighResMap    (const metric_slam_data_t& data);
    void updateRelocalization(const metric_slam_data_t& data);
    void updateMultiFloor    (const metric_slam_data_t& data);

    void transmitLPM(system::ModuleCommunicator& communicator);

    bool laserIsInitialized(int laserIndex)
    {
        return initializedLasers.find(laserIndex) != initializedLasers.end();
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_METRIC_DIRECTOR_H

