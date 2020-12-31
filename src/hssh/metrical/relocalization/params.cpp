/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.cpp
 * \author   Collin Johnson
 *
 * Definition of load_metric_relocalizer_params.
 */

#include "hssh/metrical/relocalization/params.h"
#include "utils/config_file.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

const std::string kRelocalizerHeading("MetricRelocalizerParameters");
const std::string kMaxAttemptsKey("max_relocalization_attempts");
const std::string kPositionStdKey("max_position_std_dev");
const std::string kOrientationStdKey("max_orientation_std_dev");
const std::string kMinFilterParticlesKey("min_particle_filter_samples");
const std::string kFilterTypeKey("particle_filter_type");
const std::string kMotionTypeKey("motion_model_type");
const std::string kObservationTypeKey("observation_model_type");


metric_relocalizer_params_t::metric_relocalizer_params_t(const utils::ConfigFile& config)
: filterParams(config)
, motionParams(config)
, observationParams(config)
{

    maxRelocalizationAttempts = config.getValueAsInt32(kRelocalizerHeading, kMaxAttemptsKey);
    maxPositionStdDev = config.getValueAsFloat(kRelocalizerHeading, kPositionStdKey);
    maxOrientationStdDev = config.getValueAsFloat(kRelocalizerHeading, kOrientationStdKey);

    minParticleFilterSamples = config.getValueAsInt32(kRelocalizerHeading, kMinFilterParticlesKey);

    filterType = config.getValueAsString(kRelocalizerHeading, kFilterTypeKey);
    motionModelType = config.getValueAsString(kRelocalizerHeading, kMotionTypeKey);
    observationModelType = config.getValueAsString(kRelocalizerHeading, kObservationTypeKey);

    filterParams.shouldLocalizeWhenNoMotionDetected = true;

    filterParams.kldParams.minSamples = minParticleFilterSamples;

    assert(maxRelocalizationAttempts > 0);
    assert(maxPositionStdDev > 0.0f);
    assert(maxOrientationStdDev > 0.0f);
    assert(!filterType.empty());
    assert(!observationModelType.empty());
}

}   // namespace hssh
}   // namespace vulcan
