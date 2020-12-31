/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.h
 * \author   Collin Johnson
 *
 * Declaration of params structs for metrical localization utilities.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_PARAMS_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace hssh
{

struct endpoint_observation_model_params_t
{
    float maxEndpointDistance;
    int rayStride;
    bool useCovarianceWeighting;
    bool useMovingScan;

    double occupiedLoglihood;
    double freeLoglihood;
    double unobservedLoglihood;

    endpoint_observation_model_params_t(const utils::ConfigFile& config);
};

struct gaussian_observation_model_params_t
{
    float maxEndpointDistance;
    int rayStride;

    double gaussianSigma;

    gaussian_observation_model_params_t(const utils::ConfigFile& config);
};

struct discrete_beam_model_params_t
{
    float maxRayDistance;
    int rayStride;
    bool useCovarianceWeighting;

    double hitLoglihood;
    double shortLoglihood;
    double longLoglihood;

    discrete_beam_model_params_t(const utils::ConfigFile& config);
};

struct beam_observation_model_params_t
{
    double laserVariance;   // this includes both laser noise and map discretizaion error
    double lambdaShort;
    double zHit;
    double zShort;
    double zMax;
    double zRand;

    double angleBetweenBeams;

    double minRayWeight;   // large values (~0.1) works too
    double maxRayLength;

    // occupancy weighting is used by default

    bool useDistanceWeighting;
    double minDistanceWeight;

    bool useLineWeighting;
    double defaultLineWeight;

    bool useCovarianceWeighting;

    beam_observation_model_params_t(const utils::ConfigFile& config);
};

struct observation_model_params_t
{
    std::string type;
    bool shouldDebug;

    endpoint_observation_model_params_t endpointParams;
    gaussian_observation_model_params_t gaussianParams;
    discrete_beam_model_params_t discreteParams;
    beam_observation_model_params_t beamParams;

    observation_model_params_t(const utils::ConfigFile& config);
};

struct kld_sampling_params_t
{
    std::size_t minSamples;

    float z;
    float epsilon;

    int numXBins;
    int numYBins;
    int numThetaBins;

    float xBinWidth;
    float yBinWidth;
    float thetaBinWidth;

    kld_sampling_params_t(const utils::ConfigFile& config);
};

struct particle_filter_params_t
{
    std::size_t maxUpdateAttempts;
    std::size_t initialNumSamples;   // number of samples to use when the filter is initialized
    std::size_t maxSamplesDrawn;     // need to put a cap on the number of samples drawn so that updates don't take so
                                     // long there's no hope of recovery

    double positionInitialVariance;
    double orientationInitialVariance;

    bool shouldLocalizeWhenNoMotionDetected;

    kld_sampling_params_t kldParams;

    particle_filter_params_t(const utils::ConfigFile& config);
};

struct motion_model_params_t
{
    // Parameters pulled straight from Probabilistic Robotics
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;

    float imuNoiseStdDev;

    bool saveRotationSamples;

    motion_model_params_t(const utils::ConfigFile& config);
};

/**
 * monte_carlo_localization_params_t is the params structure for the MonteCarloLocalization class. It should be
 * used. The individual params structs aren't needed by the user.
 */
struct monte_carlo_localization_params_t
{
    std::string observationModelType;
    std::string samplerType;
    std::string distributionCalculatorType;

    motion_model_params_t motionParams;
    observation_model_params_t observationParams;
    particle_filter_params_t filterParams;

    monte_carlo_localization_params_t(const utils::ConfigFile& config);
};

/**
 * localizer_params_t contains the parameters for the general localization.
 */
struct localizer_params_t
{
    std::string type;

    monte_carlo_localization_params_t monteCarloParams;

    localizer_params_t(const utils::ConfigFile& config);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_PARAMS_H
