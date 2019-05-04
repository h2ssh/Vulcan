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
* Definition of params parsers for metrical localization utilities.
*/

#include <hssh/metrical/localization/params.h>
#include <utils/config_file.h>

namespace vulcan
{
namespace hssh
{

const std::string kLocalizerHeading("LocalizerParameters");
const std::string kTypeKey("type");

const std::string kMonteCarloHeading ("MonteCarloLocalizationParameters");
const std::string kObservationTypeKey("observation_model_type");
const std::string kSamplerTypeKey    ("particle_sampler_type");
const std::string kCalculatorTypeKey ("distribution_calculator_type");

const std::string kObservationHeading("ObservationModelParameters");
const std::string kDebugKey          ("debug");

const std::string kEndpointHeading       ("EndpointObservationModelParameters");
const std::string kMaxEndpointDistanceKey("maximum_endpoint_distance_m");
const std::string kOccupiedLoglihood     ("occupied_loglihood");
const std::string kFreeLoglihood         ("free_loglihood");
const std::string kUnobservedLoglihood   ("unobserved_loglihood");
const std::string kUseMovingScanKey      ("use_moving_scan");
// using use_covariance_weight
// using ray_index_stride

const std::string kGaussianHeading ("GaussianObservationModelParameters");
const std::string kGaussianSigmaKey("sigma");
// using max_endpoint_distance
// using ray_index_stride

const std::string kDiscreteBeamHeading("DiscreteBeamObservationModelParameters");
const std::string kRayStride          ("ray_index_stride");
const std::string kHitLoglihood       ("hit_loglihood");
const std::string kShortLoglihood     ("short_loglihood");
const std::string kLongLoglihood      ("long_loglihood");
// using max_ray_length
// using use_covariance_weight

const std::string kBeamHeading         ("BeamObservationModelParameters");
const std::string kLaserVarianceKey    ("laser_variance");
const std::string kLambdaShortKey      ("lambda_short");
const std::string kZHitKey             ("z_hit");
const std::string kZShortKey           ("z_short");
const std::string kZRandKey            ("z_rand");
const std::string kZMaxKey             ("z_max");
const std::string kBeamAngleKey        ("angle_between_beams_degrees");
const std::string kMinRayWeightKey     ("min_ray_weight");
const std::string kMaxRayLengthKey     ("max_ray_length_m");
const std::string kUseDistWeightKey    ("use_distance_weighting");
const std::string kMinDistWeightKey    ("min_distance_weight");
const std::string kUseLineWeightKey    ("use_line_weight");
const std::string kDefaultLineWeightKey("default_line_weight");
const std::string kUseCovWeightKey     ("use_covariance_weight");

const std::string kMotionModelHeading("MotionModelParameters");
const std::string kAlpha1Key         ("alpha_1");
const std::string kAlpha2Key         ("alpha_2");
const std::string kAlpha3Key         ("alpha_3");
const std::string kAlpha4Key         ("alpha_4");
const std::string kImuStdKey         ("imu_noise_std_dev");
const std::string kSaveRotationKey   ("save_rotation_samples");

const std::string kParticleFilterHeading ("ParticleFilterParameters");
const std::string kMaxUpdatesKey         ("max_update_attempts");
const std::string kInitialSamplesKey     ("initial_num_samples");
const std::string kMaxSamplesKey         ("max_samples_drawn");
const std::string kPositionVarianceKey   ("initial_position_variance");
const std::string kOrientationVarianceKey("initial_orientation_variance");

const std::string kKLDHeading         ("KLDSamplingParameters");
const std::string kKLDMinSamplesKey   ("min_samples");
const std::string kKLDZKey            ("z_one_minus_delta");
const std::string kKLDEpsilonKey      ("epsilon");
const std::string kKLDNumXBinsKey     ("num_x_bins");
const std::string kKLDNumYBinsKey     ("num_y_bins");
const std::string kKLDNumThetaBinsKey ("num_theta_bins");
const std::string kKLDXBinWidthKey    ("x_bin_width");
const std::string kKLDYBinWidthKey    ("y_bin_width");
const std::string kKLDThetaBinWidthKey("theta_bin_width");


localizer_params_t::localizer_params_t(const utils::ConfigFile& config)
: type(config.getValueAsString(kLocalizerHeading, kTypeKey))
, monteCarloParams(config)
{
    assert(!type.empty());
}


monte_carlo_localization_params_t::monte_carlo_localization_params_t(const utils::ConfigFile& config)
: observationModelType      (config.getValueAsString(kMonteCarloHeading, kObservationTypeKey))
, samplerType               (config.getValueAsString(kMonteCarloHeading, kSamplerTypeKey))
, distributionCalculatorType(config.getValueAsString(kMonteCarloHeading, kCalculatorTypeKey))
, motionParams              (config)
, observationParams         (config)
, filterParams              (config)
{
    assert(!observationModelType.empty());
    assert(!samplerType.empty());
    assert(!distributionCalculatorType.empty());
}


endpoint_observation_model_params_t::endpoint_observation_model_params_t(const utils::ConfigFile& config)
: maxEndpointDistance   (config.getValueAsFloat(kEndpointHeading, kMaxEndpointDistanceKey))
, rayStride             (config.getValueAsInt32 (kEndpointHeading, kRayStride))
, useCovarianceWeighting(config.getValueAsBool(kEndpointHeading, kUseCovWeightKey))
, useMovingScan         (config.getValueAsBool(kEndpointHeading, kUseMovingScanKey))
, occupiedLoglihood     (config.getValueAsDouble(kEndpointHeading, kOccupiedLoglihood))
, freeLoglihood         (config.getValueAsDouble(kEndpointHeading, kFreeLoglihood))
, unobservedLoglihood   (config.getValueAsDouble(kEndpointHeading, kUnobservedLoglihood))
{
    assert(maxEndpointDistance > 0.0f);
    assert(rayStride > 0);
    assert(occupiedLoglihood < 0.0);
    assert(freeLoglihood < 0.0);
    assert(unobservedLoglihood < 0.0);
}


gaussian_observation_model_params_t::gaussian_observation_model_params_t(const utils::ConfigFile& config)
: maxEndpointDistance(config.getValueAsFloat(kGaussianHeading, kMaxEndpointDistanceKey))
, rayStride(config.getValueAsInt32(kGaussianHeading, kRayStride))
, gaussianSigma(config.getValueAsDouble(kGaussianHeading, kGaussianSigmaKey))
{
    assert(maxEndpointDistance > 0.0);
    assert(rayStride > 0);
    assert(gaussianSigma > 0.0);
}


discrete_beam_model_params_t::discrete_beam_model_params_t(const utils::ConfigFile& config)
: maxRayDistance        (config.getValueAsFloat (kDiscreteBeamHeading, kMaxRayLengthKey))
, rayStride             (config.getValueAsInt32 (kDiscreteBeamHeading, kRayStride))
, useCovarianceWeighting(config.getValueAsBool  (kDiscreteBeamHeading, kUseCovWeightKey))
, hitLoglihood          (config.getValueAsDouble(kDiscreteBeamHeading, kHitLoglihood))
, shortLoglihood        (config.getValueAsDouble(kDiscreteBeamHeading, kShortLoglihood))
, longLoglihood         (config.getValueAsDouble(kDiscreteBeamHeading, kLongLoglihood))
{
    assert(maxRayDistance > 0.0f);
    assert(rayStride > 0);
    assert(hitLoglihood < 0.0);
    assert(shortLoglihood < 0.0);
    assert(longLoglihood < 0.0);
}


beam_observation_model_params_t::beam_observation_model_params_t(const utils::ConfigFile& config)
: laserVariance         (config.getValueAsDouble(kBeamHeading, kLaserVarianceKey))
, lambdaShort           (config.getValueAsDouble(kBeamHeading, kLambdaShortKey))
, zHit                  (config.getValueAsDouble(kBeamHeading, kZHitKey))
, zShort                (config.getValueAsDouble(kBeamHeading, kZShortKey))
, zMax                  (config.getValueAsDouble(kBeamHeading, kZMaxKey))
, zRand                 (config.getValueAsDouble(kBeamHeading, kZRandKey))
, angleBetweenBeams     (config.getValueAsDouble(kBeamHeading, kBeamAngleKey))
, minRayWeight          (config.getValueAsDouble(kBeamHeading, kMinRayWeightKey))
, maxRayLength          (config.getValueAsDouble(kBeamHeading, kMaxRayLengthKey))
, useDistanceWeighting  (config.getValueAsBool  (kBeamHeading, kUseDistWeightKey))
, minDistanceWeight     (config.getValueAsDouble(kBeamHeading, kMinDistWeightKey))
, useLineWeighting      (config.getValueAsBool  (kBeamHeading, kUseLineWeightKey))
, defaultLineWeight     (config.getValueAsDouble(kBeamHeading, kDefaultLineWeightKey))
, useCovarianceWeighting(config.getValueAsBool  (kBeamHeading, kUseCovWeightKey))
{
}


observation_model_params_t::observation_model_params_t(const utils::ConfigFile& config)
: shouldDebug(config.getValueAsBool  (kObservationTypeKey, kDebugKey))
, endpointParams(config)
, gaussianParams(config)
, discreteParams(config)
, beamParams(config)
{
}


kld_sampling_params_t::kld_sampling_params_t(const utils::ConfigFile& config)
: minSamples   (config.getValueAsInt32(kKLDHeading, kKLDMinSamplesKey))
, z            (config.getValueAsFloat(kKLDHeading, kKLDZKey))
, epsilon      (config.getValueAsFloat(kKLDHeading, kKLDEpsilonKey))
, numXBins     (config.getValueAsInt32(kKLDHeading, kKLDNumXBinsKey))
, numYBins     (config.getValueAsInt32(kKLDHeading, kKLDNumYBinsKey))
, numThetaBins (config.getValueAsInt32(kKLDHeading, kKLDNumThetaBinsKey))
, xBinWidth    (config.getValueAsFloat(kKLDHeading, kKLDXBinWidthKey))
, yBinWidth    (config.getValueAsFloat(kKLDHeading, kKLDYBinWidthKey))
, thetaBinWidth(config.getValueAsFloat(kKLDHeading, kKLDThetaBinWidthKey))
{
}


particle_filter_params_t::particle_filter_params_t(const utils::ConfigFile& config)
: maxUpdateAttempts         (config.getValueAsUInt8(kParticleFilterHeading, kMaxUpdatesKey))
, initialNumSamples         (config.getValueAsUInt16(kParticleFilterHeading, kInitialSamplesKey))
, maxSamplesDrawn           (config.getValueAsUInt32(kParticleFilterHeading, kMaxSamplesKey))
, positionInitialVariance   (config.getValueAsDouble(kParticleFilterHeading, kPositionVarianceKey))
, orientationInitialVariance(config.getValueAsDouble(kParticleFilterHeading, kOrientationVarianceKey))
, shouldLocalizeWhenNoMotionDetected(false)
, kldParams           (config)
{
    assert(initialNumSamples > 50);
    assert(maxSamplesDrawn   > 0);
}


motion_model_params_t::motion_model_params_t(const utils::ConfigFile& config)
: alpha1        (config.getValueAsFloat(kMotionModelHeading, kAlpha1Key))
, alpha2        (config.getValueAsFloat(kMotionModelHeading, kAlpha2Key))
, alpha3        (config.getValueAsFloat(kMotionModelHeading, kAlpha3Key))
, alpha4        (config.getValueAsFloat(kMotionModelHeading, kAlpha4Key))
, imuNoiseStdDev(config.getValueAsFloat(kMotionModelHeading, kImuStdKey))
, saveRotationSamples(config.getValueAsBool(kMotionModelHeading, kSaveRotationKey))
{
    assert(alpha1 > 0.0f);
    assert(alpha2 >= 0.0f);
    assert(alpha3 > 0.0f);
    assert(alpha4 >= 0.0f);
    assert(imuNoiseStdDev > 0.0f);
}

} // namespace hssh
} // namespace vulcan
