/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_pose_sampler.h
 * \author   Jong Jin Park
 *
 * Definition of UnicyclePoseSampler for UnicycleRRTStar path planner.
 */

#include "mpepc/rrt/unicycle_pose_sampler.h"

namespace vulcan
{
namespace mpepc
{

UnicyclePoseSampler::UnicyclePoseSampler(const unicycle_pose_sampler_params_t& params) : params_(params)
{
}

pose_t
  UnicyclePoseSampler::getGoalBiasedSample(const pose_t& goalPose, const ObstacleDistanceGrid& grid, bool* isGoal) const
{
    if (drand48() < params_.goalBiasPercent / 100.0
        && grid.isPositionInFreeSpace(goalPose.toPoint(), params_.defaultDistanceThreshold)) {
        if (isGoal) {
            *isGoal = true;
        }

        std::cout << "Sampler returning goal pose\n";

        return goalPose;
    } else {
        if (isGoal) {
            *isGoal = false;
        }

        while (true) {
            pose_t randomPose = randomPoseInMap(grid);

            std::cout << "Sampled a random pose :" << randomPose << "\n";
            if (grid.isPositionInFreeSpace(randomPose.toPoint(), params_.defaultDistanceThreshold)) {
                std::cout << "Sampler returning random pose\n";
                return randomPose;
            } else {
                std::cout << "Sampled random pose is in collision\n";
            }
        }
    }
}


pose_t UnicyclePoseSampler::randomPoseInMap(const ObstacleDistanceGrid& grid) const
{
    Point<float> bottomLeft = grid.getBottomLeft();

    float randX = drand48() * grid.getWidthInMeters() + bottomLeft.x;
    float randY = drand48() * grid.getHeightInMeters() + bottomLeft.y;
    float randTheta = (drand48() - 0.5) * 2.0 * M_PI;   // in [-pi, pi)

    return pose_t(randX, randY, randTheta);
}

}   // namespace mpepc
}   // namespace vulcan
