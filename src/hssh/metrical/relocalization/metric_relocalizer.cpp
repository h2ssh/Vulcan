/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_relocalizer.cpp
* \a uthor   Collin Johnson
*
* Definition of MetricRelocalizer.
*/

#include "hssh/metrical/relocalization/metric_relocalizer.h"
#include "hssh/metrical/relocalization/debug_info.h"
#include "hssh/metrical/relocalization/filter_initializer.h"
#include "hssh/metrical/localization/monte_carlo.h"
#include "hssh/metrical/localization/particle_filter.h"
#include "hssh/metrical/localization/motion_model.h"
#include "hssh/metrical/localization/observation_model.h"
#include "hssh/metrical/localization/vanilla_particle_filter.h"
#include "hssh/metrical/occupancy_grid.h"
#include "hssh/metrical/data.h"
#include <iostream>

#define DEBUG_PROGRESS
#define DEBUG_POSE
#define DEBUG_REGION

namespace vulcan
{
namespace hssh
{

void generate_samples_at_position(const Point<float>& position, int numOrientations, double weight, std::vector<particle_t>& samples);


MetricRelocalizer::MetricRelocalizer(const metric_relocalizer_params_t& params)
: status_(RelocalizationStatus::NoTask)
, numRelocalizationAttempts_(0)
, maxRelocalizationAttempts_(params.maxRelocalizationAttempts)
, maxPositionStdDev_(params.maxPositionStdDev)
, maxOrientationStdDev_(params.maxOrientationStdDev)
{
    auto filterParams                 = params.filterParams;
    filterParams.kldParams.minSamples = params.minParticleFilterSamples;

    filter_.reset(new ParticleFilter(filterParams,
                                     std::unique_ptr<MotionModel>(new MotionModel(params.motionParams)),
                                     create_observation_model(params.observationModelType, params.observationParams),
                                     create_particle_sampler(VANILLA_PARTICLE_FILTER_TYPE),
                                     create_sample_set_distribution_calculator(VANILLA_PARTICLE_FILTER_TYPE)));
}


MetricRelocalizer::~MetricRelocalizer(void)
{
}


void MetricRelocalizer::startRelocalization(const metric_slam_data_t& data,
                                            const OccupancyGrid&      map,
                                            const FilterInitializer&  initializer)
{
    numRelocalizationAttempts_ = 0;
    status_ = RelocalizationStatus::InPrograss;
    map_ = map;

    initializeParticleFilter(data, map_, initializer);
}


relocalization_progress_t MetricRelocalizer::updateRelocalization(const metric_slam_data_t&           data,
                                                                  metric_relocalization_debug_info_t* debug)
{
    relocalization_progress_t progress;

    // Only start relocalization if there is actually something to do
    if(status_ != RelocalizationStatus::InPrograss)
    {
        progress.status = RelocalizationStatus::NoTask;
        return progress;
    }

    pose_distribution_t estimatedPose = filter_->updateFilter(data,
                                                                     map_,
                                                                     (debug ? &debug->particleFilterInfo : nullptr));
    progress.relocalizedPose = estimatedPose.toPose();

    if(isRelocalized(estimatedPose))
    {
        progress.status = RelocalizationStatus::Success;

#ifdef DEBUG_PROGRESS
        std::cout<<"DEBUG:MetricRelocalizer: Successfully relocalized in map after "<<numRelocalizationAttempts_<<" updates\n";
#endif
    }
    else if(++numRelocalizationAttempts_ > maxRelocalizationAttempts_)
    {
        progress.status = RelocalizationStatus::Failure;

#ifdef DEBUG_PROGRESS
        std::cout<<"DEBUG:MetricRelocalizer: Failed to relocalize in map.\n";
#endif
    }
    else
    {
        progress.status = RelocalizationStatus::InPrograss;
    }
    // Else still in status_

#ifdef DEBUG_POSE
    std::cout<<"DEBUG:MetricRelocalizer:Estimated pose in map:\n"<<estimatedPose.uncertainty.getMean()<<'\n'<<estimatedPose.uncertainty.getCovariance()<<'\n';
#endif

    status_ = progress.status;

    if(debug)
    {
        debug->pose             = estimatedPose;
        debug->initialParticles = initialSamples_;
    }

    return progress;
}


void MetricRelocalizer::initializeParticleFilter(const metric_slam_data_t& data,
                                                 const OccupancyGrid&      map,
                                                 const FilterInitializer&  initializer)
{
    initialSamples_ = initializer.generateInitialSamples(map, data);
    filter_->initializeWithSampleSet(initialSamples_);
}


bool MetricRelocalizer::isRelocalized(const pose_distribution_t& pose)
{
    Vector stdDevs = arma::sqrt(pose.uncertainty.getCovariance().diag());

    return (stdDevs(0) < maxPositionStdDev_) &&
           (stdDevs(1) < maxPositionStdDev_) &&
           (stdDevs(2) < maxOrientationStdDev_);
}

} // namespace hssh
} // namespace vulcan
