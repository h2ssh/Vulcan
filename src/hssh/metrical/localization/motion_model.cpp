/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_model.cpp
 * \author   Collin Johnson
 *
 * Definition of create_motion_model factory.
 */

#include "hssh/metrical/localization/motion_model.h"
#include "hssh/metrical/data.h"
#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/localization/particle.h"
#include <cassert>
#include <iostream>
#include <string>

namespace vulcan
{
namespace hssh
{

void interpolate_imu_data_at_odometry_timestamps(const std::vector<odometry_t>& odom,
                                                 std::vector<imu_data_t> imu,
                                                 std::vector<imu_data_t>& interpolatedImu);


MotionModel::MotionModel(const motion_model_params_t& params) : params_(params)
{
    assert(params_.alpha1 > 0.0f);
    //     assert(alpha2 >= 0.0f);
    assert(params_.alpha3 > 0.0f);
    //     assert(alpha4 >= 0.0f);
    assert(params_.imuNoiseStdDev > 0.0f);
}


bool MotionModel::calculateUpdateMotion(const metric_slam_data_t& data,
                                        const MultivariateGaussian& poseDistribution,
                                        int64_t deltaTime)
{
    assert(data.odometry.size() > 1);
    assert(poseDistribution.dimensions() == 3);
    assert(deltaTime >= 0);

    interpolatedImu_.clear();
    motions_.clear();

    motionTime = data.endTime;

    if (!data.imu.empty()) {
        assert(data.imu.front().timestamp == data.odometry.front().timestamp);
        assert(data.imu.back().timestamp == data.odometry.back().timestamp);
    }

    interpolate_imu_data_at_odometry_timestamps(data.odometry, data.imu, interpolatedImu_);

    auto startImuIt = interpolatedImu_.begin();
    for (std::size_t n = 1; n < data.odometry.size(); ++n) {
        auto nextImuIt = std::find_if(startImuIt, interpolatedImu_.end(), [&data, n](const imu_data_t& imu) {
            return imu.timestamp > data.odometry[n].timestamp;
        });

        motions_.emplace_back(data.odometry[n - 1], data.odometry[n], startImuIt, nextImuIt, params_);

        if (nextImuIt != startImuIt) {
            startImuIt = nextImuIt - 1;
        }
    }

    moved = false;
    for (auto& motion : motions_) {
        moved |= motion.didMove();
    }

    // For the transform, just use the first and last odom, don't worry about generating all the intermediate values,
    // which ends up being a headache and isn't worth it for the slightly more accurate covariance, as it is just used
    // for helping to weight laser scans, which is an approximate method itself
    MotionModelDistribution odomDist(data.odometry.front(),
                                     data.odometry.back(),
                                     data.imu.begin(),
                                     data.imu.end(),
                                     params_);
    transformDistribution_ = odomDist.transformDistribution();

    return moved;
}


particle_t MotionModel::moveSample(const particle_t& sample)
{
    particle_t newSample = sample;
    newSample.parent = sample.pose;

    pose_t sampledPose = sample.pose;

    if (moved) {
        for (auto& motion : motions_) {
            sampledPose = motion.applyMotion(sampledPose, MotionModelDistribution::sampled);
        }
    }

    newSample.pose = sampledPose;
    newSample.pose.timestamp = motionTime;

    return newSample;
}


pose_t MotionModel::movePoseByMean(const pose_t& pose)
{
    pose_t movedPose = pose;

    if (moved) {
        for (auto& motion : motions_) {
            movedPose = motion.applyMotion(movedPose, MotionModelDistribution::mean);
        }
    }

    movedPose.timestamp = motionTime;

    return movedPose;
}


void interpolate_imu_data_at_odometry_timestamps(const std::vector<odometry_t>& odom,
                                                 std::vector<imu_data_t> imu,
                                                 std::vector<imu_data_t>& interpolatedImu)
{
    // Nothing to do if no imu data or not enough to interpolate
    if (imu.size() < 2) {
        return;
    }

    std::size_t imuIdx = 0;

    for (std::size_t odomIdx = 0; odomIdx < odom.size(); ++odomIdx) {
        // For each odometry value, interpolate an IMU measurement at the corresponding timestamp
        for (; imuIdx < imu.size(); ++imuIdx) {
            // If the timestamp is less than, we don't need to do any interpolation so just add it to the sequence
            if (imu[imuIdx].timestamp < odom[odomIdx].timestamp) {
                interpolatedImu.push_back(imu[imuIdx]);
            }
            // If they are the same, then add it and move on to the next odometry data
            else if (imu[imuIdx].timestamp == odom[odomIdx].timestamp) {
                interpolatedImu.push_back(imu[imuIdx]);
                break;
            }
            // Once we step beyond the current odom timestamp, then need to interpolate to get a measurement at the
            // exact timestamp.
            else {
                auto newImu = interpolate_imu_data(imu[imuIdx - 1], imu[imuIdx], odom[odomIdx].timestamp);
                // Push the interpolated value
                interpolatedImu.push_back(newImu.first);
                // Change the stored IMU for the given index because the timeDelta will have changed
                imu[imuIdx] = newImu.second;

                // Break immediately because there might be more odometry data then IMU data. In this case, we'll need
                // to interpolate multiple IMU measurements for
                break;
            }
        }
    }
}

}   // namespace hssh
}   // namespace vulcan
