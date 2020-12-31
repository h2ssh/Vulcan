/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     mcl_localizer.h
 * \author   Collin Johnson
 *
 * Declaration of MonteCarloLocalization that uses Monte Carlo localization for maintaining the robot pose.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_MCL_LOCALIZER_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_MCL_LOCALIZER_H

#include "hssh/metrical/localization/localizer.h"
#include <memory>

namespace vulcan
{
namespace hssh
{

const std::string kMonteCarloType("monte-carlo");

struct particle_filter_debug_info_t;
struct monte_carlo_localization_params_t;
class ParticleFilter;

/**
 * MonteCarloLocalization implements a simple particle filter localization approach for grid maps. Odometry and IMU data
 * are used for calculating the proposal distribution and laser data is used for the observation model.
 */
class MonteCarloLocalization : public Localizer
{
public:
    /**
     * Constructor for MonteCarloLocalization.
     */
    MonteCarloLocalization(const monte_carlo_localization_params_t& params);

    /**
     * Destructor for MonteCarloLocalization.
     */
    virtual ~MonteCarloLocalization(void);

    /**
     * changeReferenceFrame changes the reference frame from which the robot pose is estimated. The transform should
     * first translate and then rotate to the new origin to get the correct pose.
     *
     * \param    referenceFrame      New reference frame for the map
     */
    void changeReferenceFrame(const pose_t& referenceFrame);

    // Localizer interface
    pose_distribution_t initializeLocalization(const metric_slam_data_t& data) override;
    void resetPoseEstimate(const pose_t& pose) override;
    pose_distribution_t updatePoseEstimate(const metric_slam_data_t& data,
                                           const OccupancyGrid& map,
                                           particle_filter_debug_info_t* debug) override;

private:
    std::unique_ptr<ParticleFilter> filter_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_MCL_LOCALIZER_H
