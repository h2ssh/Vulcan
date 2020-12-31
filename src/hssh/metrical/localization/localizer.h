/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     localizer.h
 * \author   Collin Johnson
 *
 * Declaration of Localizer interface and make_localizer factory.
 */

#ifndef HSSH_METRICAL_LOCALIZATION_LOCALIZER_H
#define HSSH_METRICAL_LOCALIZATION_LOCALIZER_H

#include <memory>

namespace vulcan
{
struct pose_t;
struct pose_distribution_t;
namespace system
{
class DebugCommunicator;
}
namespace hssh
{

class Localizer;
class OccupancyGrid;
struct localizer_params_t;
struct metric_slam_data_t;
struct particle_filter_debug_info_t;


/**
 * make_localizer creates a localizer using the provided parameters.
 *
 * \param    params          Parameters for the localizer to be created
 * \return   Desired instance of Localizer. nullptr if the instance can't be created.
 */
std::unique_ptr<Localizer> make_localizer(const localizer_params_t& params);

/**
 * Localizer is a simple interface used for localizing the robot over time. The interface allows the localization to be
 * initialized and then updated over time. Optional debugging output can be sent if requested.
 */
class Localizer
{
public:
    virtual ~Localizer(void) { }

    /**
     * initializeLocalization provides the Localizer with the initial data provided to the SLAM process.
     */
    virtual pose_distribution_t initializeLocalization(const metric_slam_data_t& data) = 0;

    /**
     * resetPoseEstimate resets the pose estimate to the provided pose. This method can be used to initialize
     * the Localizer or to reset the localization when a new place is reached, for example.
     *
     * \param    pose            Pose to which the mean of the localization estimate should be set
     * \return   New distribution for the estimated pose.
     */
    virtual void resetPoseEstimate(const pose_t& pose) = 0;

    /**
     * updatePoseEstimate performs a localization update step.
     *
     * \param    sensorData      A chunk of sensor data
     * \param    map             The map estimated from the maximum likelihood pose
     * \return   Newly estimated pose.
     */
    virtual pose_distribution_t updatePoseEstimate(const metric_slam_data_t& data,
                                                   const OccupancyGrid& map,
                                                   particle_filter_debug_info_t* debug) = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_METRICAL_LOCALIZATION_LOCALIZER_H
