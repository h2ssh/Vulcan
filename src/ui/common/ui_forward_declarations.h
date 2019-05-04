/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ui_foward_declarations.h
* \author   Collin Johnson
*
* This file contains forward declarations for the Vulcan data types used in the various
* UI modules. Rather than have a number of different files floating around, this file
* consolidates the necessary forward declarations into a single header for easy use.
*/

#ifndef UI_COMMON_UI_FORWARD_DECLARATIONS_H
#define UI_COMMON_UI_FORWARD_DECLARATIONS_H

namespace vulcan
{

struct pose_t;
struct pose_distribution_t;

class Image;
struct imu_data_t;
struct motion_state_t;
struct odometry_t;
struct polar_laser_scan_t;
struct velocity_t;

namespace robot
{
    struct commanded_velocity_t;
    struct proximity_warning_indices_t;
}

namespace hssh
{
    // local metric types
    class  LocalPerceptualMap;
    struct lpm_update_t;
    struct particle_filter_debug_info_t;
    struct metric_relocalization_debug_info_t;

    // local topo types
    class  VoronoiSkeletonGrid;
    class  GatewayGraph;
    class  LocalPlaceModel;
    class  LocalPath;
    struct AnchorPoint;
    class Gateway;
    struct gateway_normal_t;
    struct frontier_t;
    struct local_topology_place_event_t;

    // global topo types
    class  TopologicalMap;
    class  HypothesisTree;
    class  TreeOfMaps;
    class  MetricMapCache;
    struct TopologicalState;
    struct global_topo_message_t;
}

namespace laser
{
    struct tracked_object_t;
    struct laser_scan_lines_t;
}

namespace mpepc
{
    class  MetricPlannerTask;
    struct motion_controller_command_message_t;
}

namespace planner
{
    // decision planner types
    class DecisionTargetSequence;
    class DecisionPlan;

    // goal planner types
    class  GoalTarget;
    class  GoalProgress;
    class  GoalRoute;
    struct goal_debug_info_t;
}

namespace vision
{

    struct ground_plane_boundary_t;
    struct image_segment_t;
}
} // namespace vulcan

#endif // UI_COMMON_UI_FORWARD_DECLARATIONS_H
