/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     sample_set_distribution_calculator.h
 * \author   Collin Johnson
 *
 * Definition of SampleSetDistributionCalculator interface.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_SAMPLE_SET_DISTRIBUTION_CALCULATOR_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_SAMPLE_SET_DISTRIBUTION_CALCULATOR_H

#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
class MultivariateGaussian;
namespace hssh
{

class SampleSetDistributionCalculator;
struct particle_t;

/**
 * create_sample_set_distribution_calculator is a factory for creating a specified instance of
 * SampleSetDistributionCalculator.
 *
 * \param    type            Type of calculator to be created
 * \return   An instance of the desired type. nullptr otherwise.
 */
std::unique_ptr<SampleSetDistributionCalculator> create_sample_set_distribution_calculator(const std::string& type);


/**
 * SampleSetDistributionCalcuator is an interface for the strategy to be used by the
 * particle filter for collapsing the sample set into a Gaussian distribution that is used
 * as the output estimate of the robot's current pose.
 */
class SampleSetDistributionCalculator
{
public:
    virtual ~SampleSetDistributionCalculator(void) { }

    /**
     * calculateGaussianForSamples calculates a Gaussian approximation of the distribution represented
     * by the provided sample set.
     *
     * There must be at least two samples. Each sample should have a positive or zero weight.
     *
     * \param    samples         Sample set for which to calculate the distribution
     * \return   A Gaussian approximation of the distribution.
     */
    virtual MultivariateGaussian calculateGaussianForSamples(const std::vector<particle_t>& samples) const = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_SAMPLE_SET_DISTRIBUTION_CALCULATOR_H
