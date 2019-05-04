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
* Declaration of UnicyclePoseSampler for UnicycleRRTStar path planner.
*/

#ifndef UNICYCLE_POSE_SAMPLER_H
#define UNICYCLE_POSE_SAMPLER_H

#include <mpepc/grid/obstacle_distance_grid.h>
#include <core/pose.h>

namespace vulcan
{
namespace mpepc
{

struct unicycle_pose_sampler_params_t
{
    float goalBiasPercent = 10; // percentile of samples at goal pose
    float defaultDistanceThreshold = 0; // margin from obstacles when determining freespace
};


class UnicyclePoseSampler
{
public:

    /**
    * Constructor for UnicyclePoseSampler.
    */
    explicit UnicyclePoseSampler(const unicycle_pose_sampler_params_t& params);

    UnicyclePoseSampler(void) {};

    /**
    * getGoalBiasedSample samples collision-free pose in a given map, with a goal bias
    *
    * \param    goalPose        goal pose in the map
    * \param    grid            ObstacleDistanceGrid
    * \param    isGoal          (output) indicator if the sampled pose is the goal
    *
    * \return   robotPose in free space
    */
    pose_t getGoalBiasedSample(const pose_t& goalPose, const ObstacleDistanceGrid& grid, bool* isGoal) const;
//    TODO: Add this!
//    pose_t sample(const pose_t& goalPose, const ObstacleDistanceGrid& grid, const RobotCollisionModel& robotShape);

private:

    pose_t randomPoseInMap(const ObstacleDistanceGrid& grid) const;

    unicycle_pose_sampler_params_t params_;
};

} // mpepc
} // vulcan

#endif // UNICYCLE_POSE_SAMPLER_H
