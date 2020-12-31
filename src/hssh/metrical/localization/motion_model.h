/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_model.h
 * \author   Collin Johnson
 *
 * Declaration of MotionModel interface and create_motion_model factory.
 */

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_MOTION_MODEL_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_MOTION_MODEL_H

#include "core/imu_data.h"
#include "core/multivariate_gaussian.h"
#include "core/odometry.h"
#include "hssh/metrical/localization/motion_model_distribution.h"
#include <cstdint>

namespace vulcan
{
namespace hssh
{

class MotionModel;
struct particle_t;
struct metric_slam_data_t;
struct motion_model_params_t;

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
class MotionModel
{
public:
    /**
     * Constructor for OdometryMotionModel.
     *
     * \param    params          Parameters defining the uncertainty of the measurements
     */
    MotionModel(const motion_model_params_t& params);

    /**
     * calculateUpdateMotion sets up the motion model for the current update for the localization.
     * After initialization, calls to moveSample() will be made, so all distributions based on sensor data
     * should be created here.
     *
     * \param    data                Sensor values for the current update
     * \param    poseDistribution    Pose distribution from the previous time step
     * \param    deltaTime           Time change between previous and current time
     * \return   The pose transform distribution representing the uncertainty of the robot's motion.
     */
    bool calculateUpdateMotion(const metric_slam_data_t& data,
                               const MultivariateGaussian& poseDistribution,
                               int64_t deltaTime);

    /**
     * moveSample applies the motion to the provided sample and returns a new sample that
     * can be part of the proposal distribution for the particle filter.
     *
     * \param    sample          Sample to be moved
     * \return   New sample based on distribution from the motion model at the current update.
     */
    particle_t moveSample(const particle_t& sample);

    /**
     * movePoseByMean calculates a new pose by moving the previous pose by the mean of the current motion distribution.
     */
    pose_t movePoseByMean(const pose_t& pose);

    /**
     * getMotionDistribution retrieves a Gaussian distribution estimating the robot's motion on the current update.
     * The distribution is (x, y, theta) with associated covariance.
     */
    MultivariateGaussian getMotionDistribution(void) const { return transformDistribution_; }

private:
    motion_model_params_t params_;

    int64_t motionTime;
    bool moved;

    std::vector<imu_data_t> interpolatedImu_;
    std::vector<MotionModelDistribution> motions_;
    MultivariateGaussian transformDistribution_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_LOCALIZATION_MOTION_MODEL_H
