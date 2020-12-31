/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation.cpp
* \author   Jong Jin Park
* 
* Definition of NavigationTask and NavigationTaskManifold.
*/

#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/manifold/navigation.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "core/motion_state.h"
#include "utils/timestamp.h"
#include "core/float_comparison.h"

#define DEBUG_NAVIGATION_TASK
// #define DEBUG_SAFE_CHECK

namespace vulcan
{
namespace mpepc
{

bool is_safe_to_execute_target(position_t position, 
                               float safeDistance, 
                               const ObstacleDistanceGrid& map, 
                               float* minDistanceToWalls);


////////////////////// Navigation Task implementation ////////////////////////////

bool NavigationTask::setTaskParameters(const metric_planner_task_params_t& taskParams,
                                       const task_manifold_builder_params_t& builderParams)
{
    // NavigationTask doesn't currently carry around additional parameters, so always assign
    taskParams_ = taskParams.navigationTaskParams;
    manifoldParams_ = builderParams.navigationTaskManifoldParams;
    
    return true;
}


std::unique_ptr<TaskManifold> NavigationTask::createTaskManifold(void)
{
    return std::unique_ptr<TaskManifold>(new NavigationTaskManifold(target_,
                                                                    !isPositionTask_,
                                                                    taskParams_,
                                                                    manifoldParams_));
}


bool NavigationTask::isSafeToExecute(const ObstacleDistanceGrid& map) const
{
    float minDistanceToWalls;
    
    if(isPositionTask_)
    {
        for(auto position : candidateTargets_)
        {
            if(is_safe_to_execute_target(position, taskParams_.minimumSafeDistanceFromWalls, map, &minDistanceToWalls))
            {
                target_ = pose_t(position);
                return true;
            }
            else
            {
                std::cout << "IGNORED: " << position << '\n';
            }
        }

        return false;
    }
    else
    {
        return is_safe_to_execute_target(target_.toPoint(), 
                                         taskParams_.minimumSafeDistanceFromWalls, 
                                         map, 
                                         &minDistanceToWalls);
    }
}


bool NavigationTask::isComplete(const motion_state_t& state) const
{
    // Navigation task being completed means reaching the target, i.e. both the
    // distance to the target and the angle error have become small.
    float distance   = distance_between_points(state.pose.toPoint(), target_.toPoint());
    bool  isConverged = distance < taskParams_.convergenceRadius;
    
    if(isPoseTarget() && isConverged)
    {
        // don't need to check angle if the goal is positional or the distance is not converged
        float angleError = angle_diff_abs(state.pose.theta, target_.theta);
        isConverged = angleError < taskParams_.convergenceAngle;
    }
    
    return isConverged;
}


bool is_safe_to_execute_target(position_t position, 
                               float safeDistance, 
                               const ObstacleDistanceGrid& map, 
                               float* minDistanceToWalls)
{
    Point<int> targetCell = map.positionToCell(position);

    bool isCellInTheMap = map.isCellInGrid(targetCell);
    *minDistanceToWalls = map.getObstacleDistance(targetCell);
    
    bool isCellAwayFromWalls = *minDistanceToWalls > safeDistance;
    
#ifdef DEBUG_SAFE_CHECK
    std::cout<<"\nDEBUG: NavigationTask: TargetCell: ("<<targetCell.x<<" , "<<targetCell.y<<")"<<'\n';
    std::cout<<"DEBUG: NavigationTask: Is the cell in the map?: ("<<isCellInTheMap<<")"<<'\n';
    std::cout<<"DEBUG: NavigationTask: Is the cell away from walls?: ("<<isCellAwayFromWalls<<")" 
        << " Dist:" << *minDistanceToWalls << " Safe:" << safeDistance <<'\n';
#endif

    return isCellInTheMap && isCellAwayFromWalls;
}


} // mpepc
} // vulcan
