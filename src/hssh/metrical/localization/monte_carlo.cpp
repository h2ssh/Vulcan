/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     monte_carlo.cpp
* \author   Collin Johnson
*
* Definition of MonteCarloLocalization.
*/

#include "hssh/metrical/localization/monte_carlo.h"
#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/localization/observation_model.h"
#include "hssh/metrical/localization/motion_model.h"
#include "hssh/metrical/localization/particle_filter.h"
#include "hssh/metrical/localization/particle_sampler.h"
#include "hssh/metrical/localization/sample_set_distribution_calculator.h"
#include "system/debug_communicator.h"
#include "utils/timestamp.h"

// #define DEBUG_LOCALIZER_TIME

namespace vulcan
{
namespace hssh
{

MonteCarloLocalization::MonteCarloLocalization(const monte_carlo_localization_params_t& params)
: filter_(new ParticleFilter(params.filterParams,
                             std::unique_ptr<MotionModel>(new MotionModel(params.motionParams)),
                             create_observation_model(params.observationModelType, params.observationParams),
                             create_particle_sampler(params.samplerType),
                             create_sample_set_distribution_calculator(params.distributionCalculatorType)))
{
}


MonteCarloLocalization::~MonteCarloLocalization(void)
{
    // For unique_ptr
}


pose_distribution_t MonteCarloLocalization::initializeLocalization(const metric_slam_data_t& data)
{
    return filter_->initializeFilterAtPose(pose_t(0, 0, 0));
}


void MonteCarloLocalization::resetPoseEstimate(const pose_t& pose)
{
    filter_->initializeFilterAtPose(pose);
}


pose_distribution_t MonteCarloLocalization::updatePoseEstimate(const metric_slam_data_t&     data,
                                                                      const OccupancyGrid&          map,
                                                                      particle_filter_debug_info_t* debug)
{
#ifdef DEBUG_LOCALIZER_TIME
    int64_t startTime = utils::system_time_us();
#endif
    
    auto currentDistribution = filter_->updateFilter(data, map, debug);

#ifdef DEBUG_LOCALIZER_TIME
    std::cout<<"INFO:MonteCarloLocalization: filter time:"<<(utils::system_time_us()-startTime)/1000<<"ms\n";
#endif
    
    return currentDistribution;
}


void MonteCarloLocalization::changeReferenceFrame(const pose_t& referenceFrame)
{
    filter_->changeReferenceFrame(referenceFrame);
}

} // namespace hssh
} // namespace vulcan
