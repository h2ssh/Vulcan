/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
[* \file     director.cpp
* \author   Collin Johnson
*
* Definition of LocalMetricDirector, which handles organization of computation of the LPM.
*/

#include <hssh/local_metric/director.h>
#include <hssh/local_metric/lpm_io.h>
#include <hssh/local_metric/command.h>
#include <hssh/local_metric/commands/serialization.h>
#include <hssh/local_metric/pose.h>
#include <hssh/metrical/data.h>
#include <hssh/metrical/localization/localizer.h>
#include <hssh/metrical/relocalization/filter_initializer_impl.h>  // for LCM shim layer
#include <hssh/utils/save_scans.h>
#include <system/debug_communicator.h>
#include <system/module_communicator.h>
#include <utils/timestamp.h>
#include <utils/auto_mutex.h>
#include <utils/command_line.h>
#include <utils/serialized_file_io.h>
#include <cereal/types/map.hpp>
#include <fstream>
#include <iostream>

// #define DEBUG_LOG_TIME
#define DEBUG_DATAFLOW
// #define DEBUG_LOCALIZATION
// #define DEBUG_GRID_UPDATE
// #define DEBUG_RELOCALIZATION

namespace vulcan
{
namespace hssh
{

bool neighbor_is_occupied(const Point<int>& cell, const LocalPerceptualMap& lpm);


LocalMetricDirector::LocalMetricDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: params(config)
, mode(LocalMetricMode::kFullSlam)
, inputQueue(params.targetUpdateHz)
, localizer(make_localizer(params.localizationParams))
, mapper(params.mapperParams)
, highResMapper(params.highResMapperParams)
, relocalizer(params.relocalizationParams)
, multiFloor(params.multiFloorParams)
, updateCount(0)
, previousMapTransmissionTime(0ll)
, sentInitialMap(false)
, haveRelocalizationInfo(false)
, haveRelocalizationMessage(false)
, totalLocalizationTime(0)
, numLocalizations(0)
, shouldSavePoses_(commandLine.argumentExists(kSavePosesArg))
, poseTraceName_(commandLine.argumentValue(kSavePosesArg))
, mapTransmissionPeriod(params.mapTransmissionPeriodUs)
, shouldSendGlassMap(params.shouldSendGlassMap)
{
    const std::string kDefaultTraceName("local_metric_poses.log");

    if(shouldSavePoses_ && poseTraceName_.empty())
    {
        std::cerr << "WARNING: LocalMetricDirector: Requested saving poses, but no name was specified. Using default "
            << kDefaultTraceName << ". Don't forget to save it to the correct name. Otherwise, it will be overwritten "
            << "the next time you run local_metric_hssh.\n";

        poseTraceName_ = kDefaultTraceName;
    }
    else if(!poseTraceName_.empty())
    {
        std::cout << "INFO: LocalMetricDirector: Pose trace will be saved to " << poseTraceName_ << " on exit.\n";
    }
}


LocalMetricDirector::~LocalMetricDirector(void)
{
}


void LocalMetricDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    inputQueue.subscribeToData(communicator);

    communicator.subscribeTo<robot::elevator_t>(this);
    communicator.subscribeTo<std::shared_ptr<LocalMetricCommand>>(this);
    communicator.subscribeTo<motion_state_t>(this);
    communicator.subscribeTo<vulcan_lcm::save_lpm_command>(this);
    communicator.subscribeTo<vulcan_lcm::load_lpm_command>(this);
}


system::TriggerStatus LocalMetricDirector::waitForTrigger(void)
{
    processMessages(metric_slam_data_t());  // Workaround to allow processing messages while a log is paused.
        // NOTE: This workaround creates a race condition for ScanMatchingFilterInitializer because it will depend
        // whether the message is handled via this call to processMessages or a call in runUpdate. If via this call,
        // it won't work because it can't initialize since there's no laser data. However, nothing uses that message
        // at the moment, so it should be okay.

    return inputQueue.waitForData() ? system::TriggerStatus::ready : system::TriggerStatus::not_ready;
}


system::UpdateStatus LocalMetricDirector::runUpdate(system::ModuleCommunicator& communicator)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = vulcan::utils::system_time_us();
#endif
    processAvailableData();
    transmitCalculatedOutput(communicator);
#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout<<"LocalMetricDirector::run: Total update time: "<<(computationTimeUs/1000)<<"ms"<<std::endl;
#endif

    // local_metric_hssh runs continuously
    return system::UpdateStatus::running;
}


void LocalMetricDirector::shutdown(system::ModuleCommunicator& communicator)
{
    if(shouldSavePoses_)
    {
        assert(!poseTraceName_.empty());

//         if(!poseTrace_.saveToFile(poseTraceName_))
//         {
//             std::cerr << "ERROR: LocalMetricDirector: Failed to save PoseTrace as requested. Problem file:"
//                 << poseTraceName_ << '\n';
//         }
        if(utils::save_serializable_to_file(poseTraceName_, poseTrace_))
        {
            std::cout << "INFO: LocalMetricDirector: Saved poses to " << poseTraceName_ << '\n';
        }
        else
        {
            std::cerr << "ERROR: LocalMetricDirector: Failed to save poses to " << poseTraceName_ << '\n';
        }
    }
}


void LocalMetricDirector::handleData(const robot::elevator_t& elevator, const std::string& channel)
{
    utils::AutoMutex autoLock(elevatorLock);

    elevatorQueue.push_back(elevator);
}


void LocalMetricDirector::handleData(const std::shared_ptr<LocalMetricCommand>& command, const std::string& channel)
{
    commands.push(command);
}


void LocalMetricDirector::handleData(const motion_state_t& state, const std::string& channel)
{
    motionState.write(state);
}


void LocalMetricDirector::handleData(const vulcan_lcm::save_lpm_command& command, const std::string& channel)
{
    // Create a SaveMetricMapCommand and push it onto the commands queue
    commands.push(std::make_shared<SaveMetricMapCommand>("external_lcm", command.filename));
}


void LocalMetricDirector::handleData(const vulcan_lcm::load_lpm_command& command, const std::string& channel)
{
    // Load the desired LPM.
    // Create a RelocalizeInLpmCommand with the rectangle initializer around the specified initial pose
    // Push it onto the command queue for further processing
    LocalPerceptualMap relocalizationMap;
    if(!utils::load_serializable_from_file(command.filename, relocalizationMap))
    {
        std::cerr << "ERROR: LocalMetricDirector: Unable to load LPM:" << command.filename << '\n';
        return;
    }

    const int kNumPositions = 30000;
    const int kNumPosesPerPosition = 60;
    const float kRegionRadius = 1.0;

    auto initializer = std::make_shared<RegionFilterInitializer>(
        math::Rectangle<float>(Point<float>(command.initial_x - kRegionRadius,
                                                  command.initial_y - kRegionRadius),
                               Point<float>(command.initial_x + kRegionRadius,
                                                  command.initial_y + kRegionRadius)),
        kNumPositions,
        kNumPosesPerPosition
    );

    commands.push(std::make_shared<RelocalizeInLpmCommand>("external_lcm", relocalizationMap, initializer));
}


void LocalMetricDirector::processAvailableData(void)
{
    auto data = inputQueue.readData();

    if(motionState.hasData())
    {
        motionState.swapBuffers();
        currentVelocity = motionState.read().velocity;
    }

    processMessages(data);
    processLPM(data);
}


void LocalMetricDirector::transmitCalculatedOutput(system::ModuleCommunicator& communicator)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = vulcan::utils::system_time_us();
#endif

    communicator.sendMessage(currentPose);
    communicator.sendMessage(currentPoseDistribution);
    communicator.sendMessage(LocalPose{currentPoseDistribution, mapper.getLPM().getReferenceFrameIndex()});
    transmitLPM(communicator);

    communicator.sendMessage(debugInfo);

    // Send debugging laser information
    communicator.sendMessage(mapper.getMappingLaser());
    communicator.sendMessage(mapper.getReflectedLaser());

    if(haveRelocalizationInfo)
    {
        communicator.sendMessage(relocalizationInfo);
    }
#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout<<"LocalMetricDirector::transmitCalculatedOutput: time: "<<(computationTimeUs/1000)<<"ms Sent LPM="<< sentLPM <<std::endl;
#endif
    if(sentLPM)
    {
        sentInitialMap              = true;
        previousMapTransmissionTime = utils::system_time_us();
        sentLPM                     = false;
    }
}


void LocalMetricDirector::processMessages(const metric_slam_data_t& data)
{
    while(!commands.empty())
    {
        auto command = commands.front();
        commands.pop();
        std::cout << "Issuing ";
        command->print(std::cout);
        std::cout << '\n';

        command->issue(data, *localizer, mapper, relocalizer);
    }
}


void LocalMetricDirector::processLPM(const metric_slam_data_t& data)
{
    if(!laserIsInitialized(data.laser.laserId))
    {
        initializeLPM(data);
        initializedLasers.insert(data.laser.laserId);
    }
    else
    {
        updateLPM(data);
    }

#ifdef DEBUG_LOG_TIME
    timeLog<<updateCount<<' '<<((input.endTime - input.startTime)/1000)<<'\n';
#endif

    ++updateCount;
}


void LocalMetricDirector::initializeLPM(const metric_slam_data_t& data)
{
    currentPoseDistribution = localizer->initializeLocalization(data);
    currentPose = currentPoseDistribution.toPose();

    updateMap(data);
    updateHighResMap(data);
}


void LocalMetricDirector::updateLPM(const metric_slam_data_t& data)
{
    updateRelocalization(data);
    updateLocalization(data);
//     updateMultiFloor    (data);

    // Only update the map if in SLAM mode
    if(mode != LocalMetricMode::kLocalizationOnly)
    {
        updateMap(data);
    }
    else    // keep the map timestamp fresh even in localization-only mode
    {
        mapper.updateMapTime(data);
    }

    if(mode == LocalMetricMode::kHighResolutionLPM)
    {
        updateHighResMap(data);
    }

    ++updateCount;
}


void LocalMetricDirector::updateLocalization(const metric_slam_data_t& data)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = vulcan::utils::system_time_us();
#endif

    currentPoseDistribution = localizer->updatePoseEstimate(data,
                                                            mapper.getLPM(),
                                                            &(debugInfo.particleFilterInfo));
    assert(currentPoseDistribution.timestamp);
#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    totalLocalizationTime += computationTimeUs;
    ++numLocalizations;
    std::cout<<"LocalMetricDirector::run: Localization update time: "<<(computationTimeUs/1000)<<"ms. Average:"<<(totalLocalizationTime/numLocalizations/1000)<<"ms.\n";
#endif

    priorPose   = currentPose;
    currentPose = currentPoseDistribution.toPose();

    if(shouldSavePoses_)
    {
//         poseTrace_.addPose(currentPose);
        poseTrace_.emplace_back(currentPoseDistribution, mapper.getLPM().getReferenceFrameIndex());
    }

#ifdef DEBUG_LOCALIZATION
    std::cout<<"INFO:LocalMetricDirector: Pose:Mean:"<<currentPose<<" Covariance:\n"<<currentPoseDistribution.uncertainty.getCovariance()<<'\n';
#endif
}


void LocalMetricDirector::updateMap(const metric_slam_data_t& data)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = utils::system_time_us();
#endif

    mapper.updateMap(currentPoseDistribution, data);

#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout<<"LocalMetricDirector::run: Mapping update time: "<<(computationTimeUs/1000)<<"ms"<<std::endl;
#endif
}


void LocalMetricDirector::updateHighResMap(const metric_slam_data_t& data)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = utils::system_time_us();
#endif

    highResMapper.updateMap(currentPoseDistribution, data);

#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout<<"LocalMetricDirector::run: High-res mapping update time: "<<(computationTimeUs/1000)<<"ms"<<std::endl;
#endif
}


void LocalMetricDirector::updateRelocalization(const metric_slam_data_t& data)
{
#ifdef DEBUG_DATAFLOW
    int64_t computationStartTimeUs = utils::system_time_us();
#endif

    relocalization_progress_t progress = relocalizer.updateRelocalization(data, &(relocalizationInfo.info));

    // The relocalization finished and was successful
    if(progress.status == RelocalizationStatus::Success)
    {
        // Switch the map being used for SLAM
        mapper.setMap(LocalPerceptualMap(relocalizer.relocalizedMap()));
        localizer->resetPoseEstimate(progress.relocalizedPose);
    }

    haveRelocalizationInfo = progress.status == RelocalizationStatus::InPrograss;

#ifdef DEBUG_DATAFLOW
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout<<"INFO:LocalMetricDirector::run: Relocalization update time: "<<(computationTimeUs/1000)<<"ms.\n";
#endif
}


void LocalMetricDirector::updateMultiFloor(const metric_slam_data_t& data)
{
    utils::AutoMutex autoLock(elevatorLock);

    multi_floor_input_t input = { robot::elevator_t(),
                                  currentPose,
                                  mapper,
                                  relocalizer };

    multi_floor_output_t output;

    while(!elevatorQueue.empty())
    {
        input.elevator = elevatorQueue.front();

        multiFloor.processElevator(input, output);

        elevatorQueue.pop_front();
    }
}


void LocalMetricDirector::transmitLPM(system::ModuleCommunicator& communicator)
{
    int64_t currentTime = utils::system_time_us();

    if((currentTime - previousMapTransmissionTime > mapTransmissionPeriod) ||
       (lastReferenceIndex != mapper.getLPM().getReferenceFrameIndex())    ||
       !sentInitialMap)
    {
        if(mode != LocalMetricMode::kHighResolutionLPM)
        {
            communicator.sendMessage(mapper.getLPM());
        }
        else
        {
            communicator.sendMessage(highResMapper.getLPM());
        }

        if(shouldSendGlassMap)
        {
            communicator.sendMessage(mapper.getGlassMap());
        }

        lastReferenceIndex = mapper.getLPM().getReferenceFrameIndex();
        sentLPM            = true;
    }
}

} // namespace hssh
} // namespace vulcan
