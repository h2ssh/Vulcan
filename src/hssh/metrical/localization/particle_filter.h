/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     particle_filter.h
 * \author   Collin Johnson
 *
 * Declaration of ParticleFilter abstract base class for particle filters and create_particle_filter() factory.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_H

#include "core/multivariate_gaussian.h"
#include "core/pose_distribution.h"
#include "hssh/metrical/localization/kld_sample_size_calculator.h"
#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/localization/particle.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace laser
{
struct laser_scan_lines_t;
}
namespace robot
{
struct velocity_t;
}
namespace hssh
{

class OccupancyGrid;
class ParticleFilter;
class ParticleSampler;
class SampleSetDistributionCalculator;
class MotionModel;
class ObservationModel;
struct particle_filter_debug_info_t;
struct metric_slam_data_t;

/**
 * ParticleFilter is an abstract base class for a particle filter aimed at localizing the
 * robot using a laser rangefinder and occupancy grid map representation.
 *
 * The ParticleFilter uses KLD-sampling.
 */
class ParticleFilter
{
public:
    /**
     * Constructor for ParticleFilter.
     */
    ParticleFilter(const particle_filter_params_t& params,
                   std::unique_ptr<MotionModel> motion,
                   std::unique_ptr<ObservationModel> observation,
                   std::unique_ptr<ParticleSampler> sampler,
                   std::unique_ptr<SampleSetDistributionCalculator> calcuator);

    /**
     * Destructor for ParticleFilter.
     */
    virtual ~ParticleFilter(void);

    /**
     * initializeFilterAtPose initializes the particle filter with the samples distributed according
     * to the provided pose estimate.
     *
     * \param    pose            Distribution from which the initial samples should be drawn
     * \return   Initial distribution for the filter.
     */
    pose_distribution_t initializeFilterAtPose(const pose_t& pose);

    /**
     * initializeWithSampleSet initializes the particle filter with a sample set generated from somewhere.
     * Obviously, if the samples are poor, the filter won't be able to localize, but for relocalization purposes,
     * where the initial samples can be generated in a variety of ways, giving the initial sample set is handy.
     *
     * \pre      samples.size > 1
     * \param    samples         Samples to be used for the filter
     * \return   Initiali distribution for the filter
     */
    pose_distribution_t initializeWithSampleSet(const std::vector<particle_t>& samples);

    /**
     * updateFilter increments the state estimated by the filter. The filter update uses
     * the most recent laser scan and occupancy grid map, along with a motion estimate for
     * the robot since the last filter update.
     *
     * The exact form of the motion estimate depends on the motion model provided to the
     * filter when constructed.
     *
     * \param    data            Data acquired between the previous and current updates
     * \param    map             Map built from the maximum likelihood pose estimate
     * \param    debug           Location to store diagnostic debug info (output)
     * \return   Distribution centered at the maximum likelihood pose for the robot.
     */
    pose_distribution_t
      updateFilter(const metric_slam_data_t& data, const OccupancyGrid& map, particle_filter_debug_info_t* debug);

    /**
     * changeReferenceFrame transforms all the particles into a new reference frame.
     *
     * \param    referenceFrame      New frame for the particles
     */
    void changeReferenceFrame(const pose_t& referenceFrame);

private:
    particle_distribution_t prior;
    particle_distribution_t proposal;
    particle_distribution_t posterior;

    bool isInitialUpdate;

    KLDSampleSizeCalculator kldSampling;

    std::unique_ptr<MotionModel> motionModel;
    std::unique_ptr<ObservationModel> observationModel;
    std::unique_ptr<ParticleSampler> sampler;
    std::unique_ptr<SampleSetDistributionCalculator> calculator;
    MultivariateGaussian proposalDistribution;

    double sumXDiff = 0.0;
    double sumYDiff = 0.0;
    double sumThetaDiff = 0.0;
    double sumTransDiff = 0.0;
    double sumRotDiff = 0.0;
    int numUpdates = 0;

    std::ofstream deltaOut;

    pose_distribution_t currentPose;

    int64_t previousTime;
    int64_t currentTime;

    particle_filter_params_t params;


    void calculateProposalDistribution(const MultivariateGaussian& motionTransform);

    particle_t processNewSample(const particle_t& parent,
                                const laser::laser_scan_lines_t& scan,
                                const OccupancyGrid& map,
                                particle_filter_debug_info_t* debug);

    void calculatePosteriorDistribution(const laser::laser_scan_lines_t& scan,
                                        const OccupancyGrid& map,
                                        particle_filter_debug_info_t* debug);

    void calculateInitialParticleWeights(const laser::laser_scan_lines_t& scan,
                                         const OccupancyGrid& map,
                                         particle_filter_debug_info_t* debug);

    pose_distribution_t createDistributionForResetPose(const pose_t& pose);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_PARTICLE_FILTER_H
