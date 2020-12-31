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
* Declaration of GlobalMetricDirector.
*/

#ifndef HSSH_GLOBAL_METRIC_DIRECTOR_H
#define HSSH_GLOBAL_METRIC_DIRECTOR_H

#include "hssh/global_metric/messages.h"
#include "hssh/global_metric/map.h"
#include "hssh/global_metric/pose.h"
#include "utils/locked_double_buffer.h"
#include "utils/pose_trace.h"
#include "system/director.h"
#include <memory>

namespace vulcan
{
namespace utils { class CommandLine; }
namespace utils { class ConfigFile;  }
namespace hssh
{

class Mapper;
class MonteCarloLocalization;
class MetricRelocalizer;
class MetricSLAMDataQueue;


const std::string kEmulateLPMArg("emulate-lpm");
const std::string kUpdateRateArg("update-hz");
const std::string kSavePosesArg("save-poses");
const std::string kMapArg("initial-map");
const std::string kInitialRectArg("initial-rect");


/**
* GlobalMetricDirector organizes computation for the global_metric_hssh module. The state machine for the module
* follows a simple design:
*
*   1) If no map is loaded, do nothing.
*   2) If a relocalization request has been made, then try to relocalize until it finishes via failure or success.
*   3) If relocalization is successful, switch to localization mode.
*
* Once the module is localizing, it will continue localizing until it is exited.
*
* The global_metric_hssh modules accepts these command-line arguments controlling the behavior of the module:
*
*   - emulate-lpm  : send out a LocalPerceptualMap and LocalPose in addition to the GlobalMetricMap and GlobalPose
*   - update-hz    : update rate at which to run the localization (provide the Hz)
*   - save-poses   : save the estimated poses to a PoseTrace that will be saved to disk on exit (provide the filename)
*   - initial-map  : initial map in which to localize. Can be changed via the DebugUI if desired.
*   - initial-rect : initial bounding rectangle in which a relocalization attempt should be made.
*/
class GlobalMetricDirector : public system::Director
{
public:

    /**
    * Constructor for GlobalMetricDirector.
    *
    * \param    commandLine         Command line passed to the module
    * \param    config              Configuration file for the module
    */
    GlobalMetricDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    /**
    * Destructor for GlobalMetricDirector.
    */
    virtual ~GlobalMetricDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    void handleData(const global_metric_relocalization_request_message_t& message, const std::string& channel);

private:

    enum class State
    {
        kWaiting,
        kRelocalizing,
        kLocalizing
    };

    std::unique_ptr<MonteCarloLocalization> mcl_;
    std::unique_ptr<MetricRelocalizer>      relocalizer_;
    std::unique_ptr<Mapper>                 mapper_;
    std::unique_ptr<MetricSLAMDataQueue>    dataQueue_;

    GlobalMetricMap map_;
    GlobalPose      pose_;
    utils::PoseTrace poseTrace_;

    bool shouldEmulateLocalMetric_;
    bool shouldSavePoses_;
    std::string poseTraceName_;

    utils::LockedDoubleBuffer<global_metric_relocalization_request_message_t> requestMessage_;

    State   state_;
    int64_t lastTransmissionTime_;

    void processRelocalizationMessage(const metric_slam_data_t& data);
    void updateRelocalization        (const metric_slam_data_t& data, system::ModuleCommunicator& transmitter);
    void updateLocalization          (const metric_slam_data_t& data, system::ModuleCommunicator& transmitter);
    void updateMapping               (const metric_slam_data_t& data, system::ModuleCommunicator& transmitter);
};

}
}

#endif // HSSH_GLOBAL_METRIC_DIRECTOR_H
