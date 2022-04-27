/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_model_distribution.cpp
 * \author   Collin Johnson
 *
 * Definition of MotionModelDistribution.
 */

#include "hssh/metrical/localization/motion_model_distribution.h"
#include "core/imu_data.h"
#include "core/odometry.h"
#include <cassert>

// #define DEBUG_ODOMETRY

namespace vulcan
{
namespace hssh
{

const int kRot1Index = 0;
const int kTransIndex = 1;
const int kRot2Index = 2;


MotionModelDistribution::MotionModelDistribution(const odometry_t& startOdom,
                                                 const odometry_t& endOdom,
                                                 std::vector<imu_data_t>::const_iterator beginImu,
                                                 std::vector<imu_data_t>::const_iterator endImu,
                                                 const motion_model_params_t& params)
{
    bool haveImu = beginImu != endImu;

    if (haveImu) {
        if ((startOdom.timestamp != beginImu->timestamp) || (endOdom.timestamp != (endImu - 1)->timestamp)) {
            std::cout << "ERROR: Model mismatch between IMU and odometry! Odom:" << startOdom.timestamp << "->"
                      << endOdom.timestamp << " IMU:" << beginImu->timestamp << "->" << (endImu - 1)->timestamp
                      << std::endl;
        }

        assert(startOdom.timestamp == beginImu->timestamp);
        assert(endOdom.timestamp == (endImu - 1)->timestamp);
    }

    double timeDelta = 0.0;
    double imuTheta = 0.0;
    double imuVar = 0.0;
    double angularVelocity = 0.0;

    if (haveImu) {
        for (auto imuIt = beginImu + 1, prevIt = beginImu; imuIt != endImu; ++imuIt, ++prevIt) {
            int64_t timeDeltaUs = (imuIt->timeDelta > 0) ? imuIt->timeDelta : (imuIt->timestamp - prevIt->timestamp);
            timeDelta = timeDeltaUs / 1000000.0;
            angularVelocity = imuIt->rotationalVelocity[IMU_YAW_INDEX];
            imuTheta += angularVelocity * timeDelta;
            imuVar += timeDelta * timeDelta * params.imuNoiseStdDev * params.imuNoiseStdDev;
        }
        //
        //         imuTheta = wrap_to_pi((endImu-1)->orientation[IMU_YAW_INDEX] - beginImu->orientation[IMU_YAW_INDEX]);
        //         imuVar = (1 + std::abs(imuTheta)) * params.imuNoiseStdDev * params.imuNoiseStdDev;
        //         imuTheta *= -1;
    }

    double odometryTheta = angle_diff(endOdom.theta, startOdom.theta);

    double deltaX = endOdom.x - startOdom.x;
    double deltaY = endOdom.y - startOdom.y;
    double deltaTheta = haveImu ? imuTheta : odometryTheta;
    double direction = 1.0;

    double trans = std::sqrt(deltaY * deltaY + deltaX * deltaX);
    double rot1 = angle_diff(std::atan2(deltaY, deltaX), startOdom.theta);

    if (std::abs(trans) < 0.00005) {
        rot1 = 0.0;
    } else if (std::abs(rot1) > M_PI_2) {
        rot1 = angle_diff(M_PI, rot1);
        direction = -1.0;
    }

    double rot2 = angle_diff(deltaTheta, rot1);

    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (odometryTheta != 0.0);

    Vector odomMean(3);
    Matrix odomCov(3, 3);
    odomCov.zeros();

    if (moved_) {
        // Add a small amount of noise to ensure that the covariance matrix is non-singular
        const double kMinNoiseRot = 1e-8;
        const double kMinNoiseTrans = 1e-8;

        if (haveImu) {
            odomCov(kRot1Index, kRot1Index) = kMinNoiseRot + imuVar;
            odomCov(kTransIndex, kTransIndex) = kMinNoiseTrans + std::pow(params.alpha3 * trans, 2)
              + std::pow(params.alpha4 * (std::abs(rot1) + std::abs(rot2)), 2);
            odomCov(kRot2Index, kRot2Index) = kMinNoiseRot + imuVar;
        } else {
            odomCov(kRot1Index, kRot1Index) =
              kMinNoiseRot + std::pow(params.alpha1 * std::abs(rot1), 2) + std::pow(params.alpha2 * trans, 2);
            odomCov(kTransIndex, kTransIndex) = kMinNoiseTrans + std::pow(params.alpha3 * trans, 2)
              + std::pow(params.alpha4 * (std::abs(rot1) + std::abs(rot2)), 2);
            odomCov(kRot2Index, kRot2Index) =
              kMinNoiseRot + std::pow(params.alpha1 * std::abs(rot2), 2) + std::pow(params.alpha2 * trans, 2);
        }

        odomMean(kRot1Index) = rot1;
        odomMean(kTransIndex) = trans * direction;
        odomMean(kRot2Index) = rot2;

        odometryDistribution_.setDistributionStatistics(odomMean, odomCov);
        odometryDistribution_.prepareForSampling();
    }

    trans *= direction;

    Vector transformMean(3);

    transformMean(0) = trans * std::cos(deltaTheta);
    transformMean(1) = trans * std::sin(deltaTheta);
    transformMean(2) = deltaTheta;

    // Use Jacobian to propagate the error:   transError = J * odomError * J^T
    // Only need to propagate x,y error, as the theta error is just the sum of error in rot1, rot2
    Matrix jacobian(3, 3);
    jacobian.zeros();
    jacobian(0, kTransIndex) = std::cos(rot1);
    jacobian(0, kRot1Index) = -std::abs(trans) * std::sin(rot1);
    jacobian(1, kTransIndex) = std::sin(rot1);
    jacobian(1, kRot1Index) = std::abs(trans) * std::cos(rot1);
    jacobian(2, kRot1Index) = 1;
    jacobian(2, kRot2Index) = 1;

    Matrix transformCovariance = jacobian * odomCov * arma::trans(jacobian);
    transformDistribution_.setDistributionStatistics(transformMean, transformCovariance);

#ifdef DEBUG_ODOMETRY
    std::cout << "DEBUG:MotionModel:Mean:\n(" << rot1 << ',' << trans << ',' << rot2 << ")\n Cov:\n"
              << odomCov << " IMU:" << imuTheta << " Encoders:" << odometryTheta << '\n'
              << "Transform:\n"
              << transformMean << " Trans Cov:\n"
              << transformCovariance;
#endif
}


pose_t MotionModelDistribution::applyMotion(const pose_t& pose, MotionType type) const
{
    // If moved, then sample a new pose
    if (moved_) {
        // Use the mean by default
        Vector odometrySample = odometryDistribution_.getMean();

        // If a sampled pose was requested, then sample the odometry distribution
        if (type == sampled) {
            odometrySample = odometryDistribution_.sample();
        }

        float sampledRot1 = odometrySample(kRot1Index);
        float sampledTrans = odometrySample(kTransIndex);
        float sampledRot2 = odometrySample(kRot2Index);

        return pose_t(pose.x + (sampledTrans * std::cos(pose.theta + sampledRot1)),
                      pose.y + (sampledTrans * std::sin(pose.theta + sampledRot1)),
                      wrap_to_pi(pose.theta + sampledRot1 + sampledRot2));
    }
    // Otherwise, the robot must be at the same pose
    else {
        return pose;
    }
}

}   // namespace hssh
}   // namespace vulcan
