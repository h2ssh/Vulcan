/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_model_distribution.h
 * \author   Collin Johnson
 *
 * Declaration of MotionModelDistribution, which estimates motion over a short period of time.
 */

#ifndef HSSH_METRICAL_LOCALIZATION_MOTION_MODEL_DISTRIBUTION_H
#define HSSH_METRICAL_LOCALIZATION_MOTION_MODEL_DISTRIBUTION_H

#include "core/multivariate_gaussian.h"
#include "core/pose.h"
#include "hssh/metrical/localization/params.h"
#include <boost/optional.hpp>

namespace vulcan
{
struct odometry_t;
struct imu_data_t;

namespace hssh
{

/**
 * MotionModel implements the sampling-based odometry model from Table 5.6 in Probabilistic Robotics.
 * The motion model assumes the robot rotates, translates, then rotates again, given delta_rot1, delta_trans,
 * and delta_rot2. Four parameters control how uncertainty is calculated in the motion. The sampled odometry
 * becomes:
 *
 *   delta_rot1_sampled  = delta_rot1  - sample(alpha_1 * delta_rot1  + alpha_2 * delta_trans)
 *   delta_trans_sampled = delta_trans - sample(alpha_3 * delta_trans + alpha_4 * (delta_rot1 + delta_rot2))
 *   delta_rot2_sampled  = delta_rot2  - sample(alpha_1 * delta_rot2  + alpha_2 * delta_trans)
 *
 * The parameters for OdometryMotionModel are:
 *
 *   [OdometryMotionModelParameters]
 *   alpha_1 = see above
 *   alpha_2 = see above
 *   alpha_3 = see above
 *   alpha_4 = see above
 *   imu_std_dev = std dev of IMU noise
 *   save_rotation_samples = flag indicating if rotation samples when rotation > 1 degree should be saved for analysis
 *
 * A motion model is used to propagate a sample from the prior distribution, x, into
 * the proposal distribution, x', based on the supplied motion estimate of the robot
 * in the time interval [t, t'].
 *
 * To use the MotionModel, a two methods exist:
 *
 *   - bool calculateUpdateMotion(const metric_slam_data_t& sensors, const MultivariateGaussian& poseDistribution,
 * int64_t deltaTime);
 *   - particle_t moveSample(const particle_t& sample);
 *
 * initializeForCurrentUpdate() provides the most recent sensor data so the motion model can initialize the
 * distributions from which it will sample.
 *
 * moveSample() applies the motion to the provided sample and returns a new sample that
 * can be part of the proposal distribution for the particle filter.
 *
 * Optionally, the sampled rotations can be saved for the model to see the distribution. These will be saved one file
 * per update, with N samples internally. Each line of the rotation file is:
 *
 *   rot1_mean rot2_mean rot1_sample rot2_sample deltaTheta
 *
 * where deltaTheta = rot1_sample + rot2_sample
 */
class MotionModelDistribution
{
public:
    /**
     * MotionType defines the two possible types of motion to apply to a pose. Either the mean of the distribution can
     * be applied or a new pose can be sampled.
     */
    enum MotionType
    {
        mean,
        sampled,
    };

    /**
     * Constructor for MotionModelDistribution.
     *
     * If no IMU data is available, then pass beginImu == endImu.
     *
     * \param    startOdom           Odometry at start of the motion
     * \param    endOdom             Odometry at end of the motion
     * \param    beginImu            Start of IMU data associated with the odometry
     * \param    endImu              One-past-end of IMU data associated with the odometry
     * \param    params              Model parameters to use for creating the distribution
     * \pre  If IMU, first IMU timestamp == startOdom.timestamp and last IMU timestamp == endOdom.timestamp.
     */
    MotionModelDistribution(const odometry_t& startOdom,
                            const odometry_t& endOdom,
                            std::vector<imu_data_t>::const_iterator beginImu,
                            std::vector<imu_data_t>::const_iterator endImu,
                            const motion_model_params_t& params);

    /**
     * applyMotion samples a motion from the motion model and applies it to the provided pose, creating a new
     * pose based on the motion model.
     *
     * \param    pose            Pose to apply motion to
     * \param    type            Type of motion to apply to the pose
     * \return   Pose after applying motion -- either the mean or sampled.
     */
    pose_t applyMotion(const pose_t& pose, MotionType type) const;

    /**
     * didMove checks if the robot moved over the time interval represented by this distribution.
     */
    bool didMove(void) const { return moved_; }

    /**
     * transformDistribution retrieves the distribution in (x, y, theta) associated with the motion.
     */
    MultivariateGaussian transformDistribution(void) const { return transformDistribution_; }

private:
    bool moved_;
    MultivariateGaussian odometryDistribution_;
    MultivariateGaussian transformDistribution_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_METRICAL_LOCALIZATION_MOTION_MODEL_DISTRIBUTION_H
