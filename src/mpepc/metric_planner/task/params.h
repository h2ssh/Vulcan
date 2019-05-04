/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of params structs for the various task classes.
*/

#ifndef MPEPC_METRIC_PLANNER_TASK_PARAMS_H
#define MPEPC_METRIC_PLANNER_TASK_PARAMS_H

#include <mpepc/grid/params.h>
#include <mpepc/cost/social_cost.h>
#include <mpepc/cost/obstacle_cost.h>
#include <mpepc/cost/quasi_static_cost.h>
#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace mpepc
{

struct navigation_task_params_t
{
    // is safe to execute
    float minimumSafeDistanceFromWalls;

    // is complete
    float convergenceRadius;
    float convergenceAngle;

    // cost evaluation
    bool   shouldUseRRT;
    double goalNeighborhoodRadius;
    bool   shouldUseDwellTime;
    double dwellTimeWeight;
    bool   shouldUseOrientationHeuristic;
    double useGoalOrientationRadius;
    double orientationHeuristicWeight;

    // nonholonomic distance definition
    bool   shouldUseNonholonomicDistNearGoal;
    bool   shouldUseSmoothControlHeading;
    double kPhi;
    double kDelta;

    navigation_task_params_t(void) {};
    navigation_task_params_t(const utils::ConfigFile config);
};
// NOTE: The convergence criteria for the planner does not necessarily have to be
//       larger or smaller than that of the controller. It is fine to have a more
//       relaxed threshold for the planner, and it is possible for the the planner
//       to execute finer motion control by testing different inputs and estimating
//       their outcome via the simulator. The optimizer action can be understood as
//       an indirect, numerical inversion of the (arbitrarily complex) robot and
//       controller dynamics.

struct pacing_task_params_t
{
    bool  havePreferredOrientation;
    float preferredDistance;
    float preferredOrientation;

    pacing_task_params_t(void) {};
    pacing_task_params_t(const utils::ConfigFile config);
};

struct playground_task_params_t
{
    // TODO

    playground_task_params_t(void) {};
    playground_task_params_t(const utils::ConfigFile config);
};


struct metric_planner_task_params_t
{
    navigation_task_params_t navigationTaskParams;
    pacing_task_params_t     pacingTaskParams;
    playground_task_params_t playgroundTaskParams;

    metric_planner_task_params_t(void) {};
    metric_planner_task_params_t(const utils::ConfigFile& config);
};



struct navigation_task_manifold_params_t
{
    bool shouldBuildRRT;
    bool shouldDebugGrid;
    bool shouldDebugRRT;

    navigation_grid_builder_params_t navGridBuilderParams;
//     rrt_builder_params_t             rrtBuilderParams; // TODO: add this

    obstacle_distance_cost_params_t obstacleCostParams;
    quasi_static_cost_params_t quasiStaticParams;
    learned_norm_cost_params_t learnedNormParams;

    bool useLearnedNorm;

    navigation_task_manifold_params_t(void) {};
    navigation_task_manifold_params_t(const utils::ConfigFile& config);
};

struct task_manifold_builder_params_t
{
    navigation_task_manifold_params_t navigationTaskManifoldParams;

    task_manifold_builder_params_t(void) {};
    task_manifold_builder_params_t(const utils::ConfigFile& config);
};

} // mpepc
} // vulcan

#endif // MPEPC_METRIC_PLANNER_TASK_PARAMS_H
