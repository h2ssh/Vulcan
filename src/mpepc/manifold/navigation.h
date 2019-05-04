/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation.h
* \author   Collin Johnson
*
* Declaration of NavigationTaskManifold.
*/

#ifndef MPEPC_MANIFOLDS_NAVIGATION_H
#define MPEPC_MANIFOLDS_NAVIGATION_H

#include <mpepc/cost/social_cost.h>
#include <mpepc/manifold/task_manifold.h>
#include <mpepc/grid/navigation_grid.h>
#include <mpepc/grid/navigation_grid_builder.h>
#include <mpepc/metric_planner/task/params.h>
#include <hssh/local_topological/local_topo_route.h>

namespace vulcan
{
namespace mpepc
{

class NavigationTaskManifold : public TaskManifold
{
public:

    /**
    * Constructor for NavigationTaskManifold.
    *
    * \param    target              Pose target for the manifold
    * \param    isPoseTarget        Flag indicating if target includes heading or just position of the robot
    * \param    taskParams          Task-related params
    * \param    builderParams       Parameters for building the manifold
    */
    NavigationTaskManifold(const pose_t&                     target,
                           bool                                     isPoseTarget,
                           const navigation_task_params_t&          taskParams,
                           const navigation_task_manifold_params_t& builderParams);

    NavigationTaskManifold(const NavigationGrid&                    navGrid,
                           const navigation_task_params_t&          taskParams,
                           const navigation_task_manifold_params_t& builderParams);

    // TaskManifold interface
    pose_t target(void) const override { return target_; };
    void update(const planning_environment_t& env,
                const std::vector<dynamic_object_trajectory_t>& trajectories) override;
    std::vector<dynamic_object_trajectory_t> createTaskSpecificObjects(const planning_environment_t& env) override;
    void calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory,
                                                float timestep,
                                                std::size_t stride,
                                                std::vector<float>& rewards) const override;
    double calculateHeuristicCost(const robot_trajectory_info_t& robotTrajectoryInfo) const override;
    void sendDebugInfo(system::ModuleCommunicator& transmitter) override;

private:

    CostMap costMap_;       // cost map to maintain information about obstacle distances and quasi-static distance
    NavigationGrid navGrid_;
    NavigationGridBuilder navGridBuilder_;

    std::vector<Point<float>> quasiStaticPoints_;
    std::vector<std::size_t> quasiStaticIndices_;

    pose_t target_;
    bool isPoseTarget_;
    pose_t routeTarget_;     // target of interest along current route. Without topo map, it's same as target_.
    hssh::LocalTopoRoute topoRoute_;
    std::vector<Line<double>> routeGateways_;

    float minDistanceToWallsFromTarget_;
    bool  isVeryCloseToGoal_;
    bool  isApproachingGoal_;
    bool  isUsingNavGrid_;
    bool  isUsingNavGradient_;
    bool haveNewNavGrid_;

    int64_t robotTrajectoryTimestamp_;
    float   initialHeadingError_;

    int64_t debugDataUpdatePeriodUs_;
    int64_t lastDebugDataUpdateTimeUs_;

    learned_norm_info_t learnedNormDebug;

    navigation_task_params_t          taskParams_;
    navigation_task_manifold_params_t builderParams_;


    double piecewiseNegativeReward(const motion_state_t& startState,
                                   const motion_state_t& endState,
                                   float timestep) const;
    double getCostToGo(const motion_state_t& state) const;

    void updateTopoRoute(const planning_environment_t& env);
    void updateNavigationGrid(const planning_environment_t& env,
                              const std::vector<dynamic_object_trajectory_t>& trajectories);
    void updateStateMachine(const ObstacleDistanceGrid&, const motion_state_t& state);
    void createQuasiStaticPoints(const std::vector<dynamic_object_trajectory_t>& trajectories);

    double getCostToGoFromNavGrid(const motion_state_t& state) const;

    double computeNavGradient(const pose_t& pose) const;
};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_MANIFOLDS_NAVIGATION_H
