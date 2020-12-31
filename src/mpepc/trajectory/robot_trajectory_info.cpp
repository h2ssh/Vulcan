/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     robot_trajectory_info.cpp
 * \author   Collin Johnson
 *
 * Definition of methods for robot_trajectory_info_t and robot_trajectory_debug_info_t.
 */

#include "mpepc/trajectory/robot_trajectory_info.h"

namespace vulcan
{
namespace mpepc
{

void robot_trajectory_info_t::getPoses(std::vector<pose_t>& poses) const
{
    for (auto stateIt = states.begin(); stateIt != states.end(); stateIt++) {
        poses.push_back(stateIt->pose);
    }
}


void robot_trajectory_info_t::clear(void)
{
    states.clear();
    commands.clear();
    controlLawCoordinates.clear();
    piecewiseSurvivability.clear();
    piecewiseRawProgress.clear();
    piecewiseCollisionCost.clear();
    piecewiseActionCost.clear();
    distanceToStaticObj.clear();
    distanceToDynamicObj.clear();
}


robot_trajectory_debug_info_t::robot_trajectory_debug_info_t(const robot_trajectory_info_t& trjInfo)
{
    assign(trjInfo);
}


void robot_trajectory_debug_info_t::assign(const robot_trajectory_info_t& trjInfo)
{
    poses.clear();
    trjInfo.getPoses(poses);

    motionTarget = trjInfo.motionTarget;
    piecewiseSurvivability = trjInfo.piecewiseSurvivability;
    piecewiseRawProgress = trjInfo.piecewiseRawProgress;
    piecewiseCollisionCost = trjInfo.piecewiseCollisionCost;
    piecewiseActionCost = trjInfo.piecewiseActionCost;
    expectedCost = trjInfo.expectedCost;
    totalSurvivability = trjInfo.totalSurvivability;
    expectedProgress = trjInfo.expectedProgress;
    expectedCollisionCost = trjInfo.expectedCollisionCost;
    expectedActionCost = trjInfo.expectedActionCost;
    hasCollision = trjInfo.hasStaticCollision || trjInfo.hasDynamicCollision;
}

}   // namespace mpepc
}   // namespace vulcan
