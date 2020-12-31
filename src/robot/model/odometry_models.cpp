/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     odometry_models.cpp
 * \author   Collin Johnson
 *
 * Implementation of InverseOdometryModel.
 */

#include "robot/model/odometry_models.h"
#include "core/odometry.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace robot
{

const std::string kOdometryModelHeading("OdometryMotionModelParameters");
const std::string kAlpha1Key("alpha_1");
const std::string kAlpha2Key("alpha_2");
const std::string kAlpha3Key("alpha_3");
const std::string kAlpha4Key("alpha_4");


odometry_model_params_t::odometry_model_params_t(const utils::ConfigFile& config)
: alpha1(config.getValueAsDouble(kOdometryModelHeading, kAlpha1Key))
, alpha2(config.getValueAsDouble(kOdometryModelHeading, kAlpha2Key))
, alpha3(config.getValueAsDouble(kOdometryModelHeading, kAlpha3Key))
, alpha4(config.getValueAsDouble(kOdometryModelHeading, kAlpha4Key))
{
}


InverseOdometryModel::InverseOdometryModel(const odometry_model_params_t& params) : params_(params)
{
}


pose_distribution_t InverseOdometryModel::predictPose(const pose_distribution_t& previousPose,
                                                      const motion_model_data_t& previousData,
                                                      const motion_model_data_t& currentData)
{
    // Can't do anything if there isn't any odometry data
    if (!previousData.odometry || !currentData.odometry) {
        return previousPose;
    }

    const auto& previousOdom = *(previousData.odometry);
    const auto& currentOdom = *(currentData.odometry);

    double rot1 =
      angle_diff(std::atan2(currentOdom.y - previousOdom.y, currentOdom.x - previousOdom.x), previousOdom.theta);
    double trans = std::sqrt(std::pow(currentOdom.y - previousOdom.y, 2) + std::pow(currentOdom.x - previousOdom.x, 2));

    double direction = 1.0;

    // Check if not moving or if moving backward to avoid the rot1 value being a huge value like pi, when the robot
    // hasn't actually moved, it's just driving backward.
    // Yes, this means that a rotation of more than pi/2 is not supported in a single update, but rotating that fast
    // requires spinning around 600 rpms.
    if (std::abs(trans) < 0.00005) {
        rot1 = 0.0;
    } else if (std::abs(rot1) > M_PI / 2.0) {
        rot1 = angle_diff(M_PI, rot1);
        direction = -1.0;
    }

    double rot2 = wrap_to_pi(currentOdom.theta - previousOdom.theta - rot1);

    Matrix gT(3, 3);
    gT.eye();
    gT(0, 2) = -trans * std::sin(previousPose.theta + rot1);
    gT(1, 2) = trans * std::cos(previousPose.theta + rot1);

    Matrix vT(3, 3);
    vT.zeros();
    vT(0, 0) = -trans * std::sin(previousPose.theta + rot1);
    vT(0, 1) = std::cos(previousPose.theta + rot1);
    vT(1, 0) = trans * std::cos(previousPose.theta + rot1);
    vT(1, 1) = std::sin(previousPose.theta + rot1);
    vT(2, 0) = 1.0;
    vT(2, 2) = 1.0;

    Matrix mT(3, 3);
    mT.eye();
    mT *= 1e-7;
    mT(0, 0) = params_.alpha1 * std::abs(rot1) + params_.alpha2 * trans;
    mT(1, 1) = params_.alpha3 * trans + params_.alpha4 * (std::abs(rot1) + std::abs(rot2));
    mT(2, 2) = params_.alpha1 * std::abs(rot2) + params_.alpha2 * trans;

    trans *= direction;

    pose_distribution_t predictedPose;
    predictedPose.x = previousPose.x + trans * std::cos(previousPose.theta + rot1);
    predictedPose.y = previousPose.y + trans * std::sin(previousPose.theta + rot1);
    predictedPose.theta = wrap_to_pi(previousPose.theta + rot1 + rot2);

    predictedPose.uncertainty.setDistributionStatistics(
      Vector({predictedPose.x, predictedPose.y, predictedPose.theta}),
      gT * previousPose.uncertainty.getCovariance() * gT.t()
        + vT * mT * vT.t());   // Non-linear uncertainty propagation for EKF update

    predictedPose.timestamp = currentOdom.timestamp;

    return predictedPose;
}

}   // namespace robot
}   // namespace vulcan
