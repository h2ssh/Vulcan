/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     known_pose_localizer.cpp
* \author   Collin Johnson
* 
* Definition of KnownPoseLocalizer.
*/

#include "hssh/metrical/localization/known_pose_localizer.h"
#include "hssh/metrical/data.h"
#include "core/pose_distribution.h"

namespace vulcan
{
namespace hssh
{
    
pose_t odometry_to_pose(const odometry_t& odom);
pose_distribution_t pose_to_distribution(const pose_t& pose, int64_t timestamp);
    
    
pose_distribution_t KnownPoseLocalizer::initializeLocalization(const metric_slam_data_t& data) 
{
    if(!data.odometry.empty())
    {
        initialPose_ = odometry_to_pose(data.odometry.back());
    }
    
    return pose_to_distribution(pose_t(0, 0, 0), data.endTime);
}
    

void KnownPoseLocalizer::resetPoseEstimate(const pose_t& pose)
{
    initialPose_ = pose;
}


pose_distribution_t KnownPoseLocalizer::updatePoseEstimate(const metric_slam_data_t& data, 
                                                                  const OccupancyGrid& map,
                                                                  particle_filter_debug_info_t* debug)
{
    pose_t pose(data.endTime, 0, 0, 0);
    
    // If no data, then the pose isn't required to be anything meaningful
    if(data.odometry.empty())
    {
        std::cerr << "ERROR: KnownPoseLocalizer: No odometry data available.\n";
    }
    else
    {
        // Just set the mean to the latest piece of odometry data available
        pose = odometry_to_pose(data.odometry.back()).transformToNewFrame(initialPose_);
    }
    
    return pose_to_distribution(pose, data.endTime);
}


pose_t odometry_to_pose(const odometry_t& odom)
{
    return pose_t(odom.x, odom.y, odom.theta);
}


pose_distribution_t pose_to_distribution(const pose_t& pose, int64_t timestamp)
{
    MultivariateGaussian poseDist(3);
    poseDist(0, 0) = 1e-5;
    poseDist(0, 1) = 0;
    poseDist(0, 2) = 0;
    poseDist(1, 0) = 0;
    poseDist(1, 1) = 1e-5;
    poseDist(1, 2) = 0;
    poseDist(2, 0) = 0;
    poseDist(2, 1) = 0;
    poseDist(2, 2) = 1e-5;
    
    poseDist[0] = pose.x;
    poseDist[1] = pose.y;
    poseDist[2] = pose.theta;
    
    return pose_distribution_t(timestamp, poseDist);
}

}
}
