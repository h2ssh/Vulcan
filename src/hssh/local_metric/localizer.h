/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     localizer.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of Localizer abstract base class for subclasses that perform robot localization and a create_localizer()
 * factory.
 */

#ifndef HSSH_LOCAL_METRIC_LOCALIZATION_LOCALIZER_H
#define HSSH_LOCAL_METRIC_LOCALIZATION_LOCALIZER_H

#include "core/pose.h"
#include "core/pose_distribution.h"
#include <boost/shared_ptr.hpp>

namespace vulcan
{
namespace robot
{
struct velocity_t;
}
namespace hssh
{

class Localizer;
class LocalPerceptualMap;
struct metric_slam_data_t;
struct localization_params_t;
struct local_metric_debug_info_t;

/**
 * create_localizer creates a Localizer instance based on the provided string descriptor.
 *
 * \param    localizerName           Name of the Localizer to create
 * \param    params                  Generic lpm_params_t that contains the parameters for the specific Localizer
 * \return   An instance of Localizer to be used for localizing the robot. If no Localizer is defined with the given
 *           name, a null pointer is returned.
 */
boost::shared_ptr<Localizer> create_localizer(const std::string& localizerName, const localization_params_t& params);

/**
 * Localizer is an abstract base class that represents any of a number of potential localization methods for
 * the LPM. The Localizer takes all available sensor data and the map and produces an updated pose estimate
 * for the robot.
 *
 * A Localizer is created using a static factory declared in this header file.
 */
class Localizer
{
public:
    virtual ~Localizer(void) { }

    /**
     * resetPoseEstimate resets the pose estimate to the provided pose. This method can be used to initialize
     * the Localizer or to reset the Localizer when a new place is reached, for example.
     *
     * \param    pose            Pose to which the mean of the localization estimate should be set
     */
    virtual void resetPoseEstimate(const pose_t& pose) = 0;

    /**
     * updatePoseEstimate performs a localization update step.
     *
     * \param    sensorData      A chunk of sensor data
     * \param    velocity        Current robot velocity
     * \param    map             The map estimated from the maximum likelihood pose
     * \param    debug           Debugging information about the localization state
     */
    virtual void updatePoseEstimate(const metric_slam_data_t& sensorData,
                                    const velocity_t& velocity,
                                    const LocalPerceptualMap& map,
                                    local_metric_debug_info_t& debug) = 0;

    /**
     * changeReferenceFrame changes the reference frame from which the robot pose is estimated. The transform should
     * first translate and then rotate to the new origin to get the correct pose.
     *
     * \param    referenceFrame      New reference frame for the map
     */
    virtual void changeReferenceFrame(const pose_t& referenceFrame) = 0;

    /**
     * getPose retrieves the current maximum likelihood pose estimate.
     */
    pose_t getPose(void) const { return currentPose; }

    /**
     * getPoseDistribution retrieves the full distribution of the maximum likelihood pose estimate.
     */
    pose_distribution_t getPoseDistribution(void) const { return currentPoseDistribution; }

protected:
    Localizer(void) { }

    pose_t currentPose;
    pose_distribution_t currentPoseDistribution;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_METRIC_LOCALIZATION_LOCALIZER_H
