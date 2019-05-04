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
* Definition of GlobalMetricDirector.
*/

#include <hssh/global_metric/director.h>
#include <hssh/global_metric/debug_info.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/pose.h>
#include <hssh/metrical/localization/monte_carlo.h>
#include <hssh/metrical/localization/params.h>
#include <hssh/metrical/relocalization/metric_relocalizer.h>
#include <hssh/metrical/relocalization/params.h>
#include <hssh/metrical/relocalization/filter_initializer_impl.h> // for serialization
#include <hssh/metrical/relocalization/scan_matching_initializer.h> // for serialization
#include <hssh/metrical/data.h>
#include <hssh/metrical/data_queue.h>
#include <hssh/metrical/mapping/mapper.h>
#include <system/module_communicator.h>
#include <utils/timestamp.h>
#include <utils/command_line.h>
#include <utils/config_file_utils.h>
#include <utils/serialized_file_io.h>
#include <cereal/types/polymorphic.hpp>

namespace vulcan
{
namespace hssh
{

GlobalMetricDirector::GlobalMetricDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: mcl_(new MonteCarloLocalization(monte_carlo_localization_params_t(config)))
, relocalizer_(new MetricRelocalizer(metric_relocalizer_params_t(config)))
, mapper_(new Mapper(load_mapper_params(config, "MapperParameters")))
, dataQueue_(new MetricSLAMDataQueue(std::atof(commandLine.argumentValue(kUpdateRateArg).c_str())))
, shouldEmulateLocalMetric_(commandLine.argumentExists(kEmulateLPMArg))
, shouldSavePoses_(commandLine.argumentExists(kSavePosesArg))
, poseTraceName_(commandLine.argumentValue(kSavePosesArg))
, state_(State::kWaiting)
, lastTransmissionTime_(0)
{
    const std::string kDefaultTraceName("global_metric_poses.log");

    if(shouldSavePoses_ && poseTraceName_.empty())
    {
        std::cerr << "WARNING: GlobalMetricDirector: Requested saving poses, but no name was specified. Using default "
            << kDefaultTraceName << ". Don't forget to save it to the correct name. Otherwise, it will be overwritten "
            << "the next time you run global_metric_hssh.\n";

        poseTraceName_ = kDefaultTraceName;
    }
    else if(!poseTraceName_.empty())
    {
        std::cout << "INFO: GlobalMetricDirector: Pose trace will be saved to " << poseTraceName_ << " on exit.\n";
    }

    if(commandLine.argumentExists(kMapArg) && commandLine.argumentExists(kInitialRectArg))
    {
        auto mapName = commandLine.argumentValue(kMapArg);
        auto mapRect = utils::create_rectangle_from_string(commandLine.argumentValue(kInitialRectArg));

        std::cout << "INFO: GlobalMetricDirector: Initializing relocalization in " << mapName << " inside "
            << mapRect << '\n';

        bool haveMap = true;
        if(mapName.find(".gmm") != std::string::npos)
        {
            haveMap = utils::load_serializable_from_file(mapName, map_);
        }
        else if(mapName.find(".lpm") != std::string::npos)
        {
            // If can't load the GMM, then see if it can be interpreted as an LPM.
            hssh::LocalPerceptualMap lpm;
            haveMap = utils::load_serializable_from_file(mapName, lpm);
            if(haveMap)
            {
                map_ = GlobalMetricMap(mapName, lpm);
            }
        }

        // If a map is found, then create a relocalization message for it
        if(haveMap)
        {
            hssh::global_metric_relocalization_request_message_t msg;
            msg.initializer = std::make_shared<hssh::RegionFilterInitializer>(mapRect, 30000, 60);
            msg.map         = map_;
            handleData(msg, "INTERNAL");
        }
        else
        {
            std::cerr << "ERROR: Failed to load initial map: " << mapName << '\n';
        }
    }
}


GlobalMetricDirector::~GlobalMetricDirector(void)
{
    // For std::unique_ptr
}


void GlobalMetricDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    dataQueue_->subscribeToData(communicator);
    communicator.subscribeTo<global_metric_relocalization_request_message_t>(this);
}


system::TriggerStatus GlobalMetricDirector::waitForTrigger(void)
{
    bool haveData = false;
    for(int n = 0; ++n < 20 && !haveData;)
    {
        haveData = dataQueue_->waitForData();
    }

    return haveData ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus GlobalMetricDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    // Read the data to be used for this update of the global metric hssh
    // If a new relocalization request has arrived, start the new relocalization process
    // If relocalizing, continue doing so
    // If localizing, continue doing so

    auto data = dataQueue_->readData();

    if(requestMessage_.hasData())
    {
        processRelocalizationMessage(data);
    }

    switch(state_)
    {
    case State::kRelocalizing:
        updateRelocalization(data, communicator);
        break;

    case State::kLocalizing:
        updateLocalization(data, communicator);
        updateMapping(data, communicator);
        break;

    case State::kWaiting:
        // Nothing to do when waiting for a message to arrive
        break;
    }

    // Always ready to keep running in global_metric_hssh
    return system::UpdateStatus::running;
}


void GlobalMetricDirector::shutdown(system::ModuleCommunicator& communicator)
{
    if(shouldSavePoses_)
    {
        assert(!poseTraceName_.empty());

        if(!poseTrace_.saveToFile(poseTraceName_))
        {
            std::cerr << "ERROR: GlobalMetricDirector: Failed to save PoseTrace as requested. Problem file:"
                << poseTraceName_ << '\n';
        }
    }
}


void GlobalMetricDirector::handleData(const global_metric_relocalization_request_message_t& message, const std::string& channel)
{
    requestMessage_.write(message);
}


void GlobalMetricDirector::processRelocalizationMessage(const metric_slam_data_t& data)
{
    assert(requestMessage_.hasData());
    requestMessage_.swapBuffers();

    auto& msg = requestMessage_.read();
    map_ = msg.map;

    relocalizer_->startRelocalization(data, map_, *msg.initializer);
    state_ = State::kRelocalizing;

    std::cout << "INFO: GlobalMetricDirector: Beginning relocalization in new global metric map...\n";
}


void GlobalMetricDirector::updateRelocalization(const metric_slam_data_t& data, system::ModuleCommunicator& transmitter)
{
    assert(state_ == State::kRelocalizing);

    global_metric_relocalization_info_t debugInfo;

    auto progress = relocalizer_->updateRelocalization(data, &(debugInfo.info));

    if(progress.status == RelocalizationStatus::Success)
    {
        mcl_->resetPoseEstimate(progress.relocalizedPose);
        state_ = State::kLocalizing;

        if(shouldEmulateLocalMetric_)
        {
            mapper_->setMap(LocalPerceptualMap(map_));
        }

        std::cerr << "SUCCESS: GlobalMetricDirector: Relocalized in map " << map_.name() << " at " << progress.relocalizedPose << '\n';
    }
    else if(progress.status == RelocalizationStatus::Failure)
    {
        std::cerr << "ERROR: GlobalMetricDirector: Failed to relocalize in map : " << map_.name() << '\n';
    }

    transmitter.sendMessage(debugInfo);
}


void GlobalMetricDirector::updateLocalization(const metric_slam_data_t& data, system::ModuleCommunicator& transmitter)
{
    assert(state_ == State::kLocalizing);

    global_metric_localization_info_t debugInfo;
    auto pose = mcl_->updatePoseEstimate(data, map_, &(debugInfo.info));
    pose_ = GlobalPose(pose, map_.id());

    transmitter.sendMessage(pose_);
    transmitter.sendMessage(debugInfo);

    if(shouldEmulateLocalMetric_)
    {
        transmitter.sendMessage(LocalPose(pose, mapper_->getLPM().getReferenceFrameIndex()));
        transmitter.sendMessage(pose.toPose());
        transmitter.sendMessage(pose);
    }

    if(shouldSavePoses_)
    {
        poseTrace_.addPose(pose.toPose());
    }
}


void GlobalMetricDirector::updateMapping(const metric_slam_data_t& data, system::ModuleCommunicator& transmitter)
{
    assert(state_ == State::kLocalizing);

    const int64_t kMapTransmissionPeriodUs = 1000000;

    if(shouldEmulateLocalMetric_)
    {
        mapper_->updateMap(pose_.poseDistribution(), data);
    }

    if(utils::system_time_us() - lastTransmissionTime_ > kMapTransmissionPeriodUs)
    {
        lastTransmissionTime_ = utils::system_time_us();

        if(shouldEmulateLocalMetric_)
        {
            transmitter.sendMessage(mapper_->getLPM());
        }
        else
        {
            transmitter.sendMessage(map_);
        }
    }
}

} // namespace hssh
} // namespace vulcan
