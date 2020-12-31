/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ui_params.cpp
* \author   Collin Johnson
*
* Definition of functions to turn the contents of a utils::ConfigFile into
* the appropriate UI params structs.
*/

#include "ui/common/ui_params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"
#include <iostream>

namespace vulcan
{
namespace ui
{

// Constants defining the parameter strings in the configuration file
const std::string LPM_DISPLAY_HEADING       ("LocalMetricDisplayParameters");
const std::string FRONT_LASER_COLOR_KEY     ("front_laser_color");
const std::string BACK_LASER_COLOR_KEY      ("back_laser_color");
const std::string WARNING_LASER_COLOR_KEY   ("warning_laser_color");
const std::string CRITICAL_LASER_COLOR_KEY  ("critical_laser_color");
const std::string INTENSITY_LASER_COLOR_KEY ("intensity_laser_color");
const std::string EXTRACTED_LINES_COLOR_KEY ("extracted_lines_color");
const std::string OCCUPIED_COLOR_KEY        ("occupied_cell_color");
const std::string DYNAMIC_COLOR_KEY         ("dynamic_cell_color");
const std::string LIMITED_VIS_COLOR_KEY     ("limited_visibility_cell_color");
const std::string HAZARD_COLOR_KEY          ("hazard_cell_color");
const std::string QUASI_STATIC_COLOR_KEY    ("quasi_static_cell_color");
const std::string ROBOT_COLOR_KEY           ("robot_color");
const std::string TRACE_COLOR_KEY           ("robot_trace_color");
const std::string ODOM_TRACE_COLOR_KEY      ("odometry_trace_color");
const std::string PARTICLES_COLOR_KEY       ("particles_color");
const std::string CIRCLE_COLOR_KEY          ("target_circle_color");
const std::string ARROW_COLOR_KEY           ("target_arrow_color");
const std::string METRIC_PATH_COLOR_KEY     ("metric_path_color");
const std::string TRACE_LENGTH_KEY          ("max_trace_length");
const std::string LOCAL_METRIC_MESSAGE_KEY  ("local_metric_hssh_message_output_channel");
const std::string METRIC_PATH_KEY           ("metric_waypoint_path_channel");
const std::string REACHED_CIRCLE_COLOR_KEY  ("reached_target_circle_color");
const std::string REACHED_ARROW_COLOR_KEY   ("reached_target_arrow_color");
const std::string ACQUIRING_TARGET_COLOR_KEY("acquiring_target_color");
const std::string ACTIVE_TARGET_COLOR_KEY   ("active_target_color");
const std::string INACTIVE_TARGET_COLOR_KEY ("inactive_target_color");

const std::string LOCAL_TOPO_DISPLAY_HEADING("LocalTopoDisplayParameters");
const std::string FRONTIER_CELL_COLOR_KEY   ("frontier_color");
const std::string GATEWAY_CELL_COLOR_KEY    ("gateway_color");
const std::string JUNCTION_COLOR_KEY        ("junction_color");
const std::string DEAD_END_COLOR_KEY        ("dead_end_color");
const std::string SKELETON_CELL_COLOR_KEY   ("skeleton_cell_color");
const std::string REDUCED_CELL_COLOR_KEY    ("reduced_cell_color");
const std::string SOURCE_CELL_COLOR_KEY     ("source_color");
const std::string EXPLORED_COLOR_KEY        ("explored_color");
const std::string UNKNOWN_AREA_COLOR_KEY    ("unknown_area_color");
const std::string LOCAL_PATH_COLOR_KEY      ("local_path_color");
const std::string LOCAL_DECISION_COLOR_KEY  ("local_decision_color");
const std::string LOCAL_DEST_COLOR_KEY      ("local_destination_color");
const std::string LOCAL_PATH_DEST_COLOR_KEY ("local_path_dest_color");
const std::string LOCAL_PATH_DECI_COLOR_KEY ("local_path_decision_color");
const std::string LOCAL_DEST_DECI_COLOR_KEY ("local_dest_decision_color");
const std::string GATEWAY_END_COLOR_KEY     ("gateway_endpoint_color");
const std::string LOCAL_PATH_START_COLOR_KEY("local_path_start_color");
const std::string LOCAL_PATH_END_COLOR_KEY  ("local_path_end_color");
const std::string ISOVIST_RAY_COLOR_KEY     ("isovist_ray_color");
const std::string ISOVIST_FIELD_COLORS_KEY  ("isovist_field_colors");

const std::string GLOBAL_TOPO_DISPLAY_HEADING("GlobalTopoDisplayParameters");
const std::string GLOBAL_PLACE_COLOR_KEY     ("place_color");
const std::string GLOBAL_PATH_COLOR_KEY      ("path_color");
const std::string GLOBAL_FRONTIER_COLOR_KEY  ("frontier_color");
const std::string GLOBAL_LOCATION_COLOR_KEY  ("global_location_color");
const std::string HYP_TREE_NODE_COLOR_KEY    ("hypothesis_tree_node_color");
const std::string HYP_TREE_EDGE_COLOR_KEY    ("hypothesis_tree_edge_color");
const std::string HYP_TREE_BEST_COLOR_KEY    ("hypothesis_tree_best_node_color");
const std::string HYP_TREE_HOVER_COLOR_KEY   ("hypothesis_tree_hover_node_color");
const std::string HYP_TREE_SEL_A_COLOR_KEY   ("hypothesis_tree_selected_node_a_color");
const std::string HYP_TREE_SEL_B_COLOR_KEY   ("hypothesis_tree_selected_node_b_color");
const std::string HYP_TREE_SEL_C_COLOR_KEY   ("hypothesis_tree_selected_node_c_color");
const std::string CORRECT_MAP_CHANNEL_KEY    ("correct_global_topo_map_output_channel");

const std::string METRIC_PLANNER_DISPLAY_HEADING("MetricPlannerDisplayParameters");
const std::string TRAJECTORY_COLOR_RED_KEY      ("trajectory_color_red");
const std::string TRAJECTORY_COLOR_BLUE_KEY     ("trajectory_color_blue");
const std::string TRAJECTORY_COLOR_GREEN_KEY    ("trajectory_color_green");
const std::string ROBOT_COLOR_RED_KEY           ("robot_color_red");
const std::string ROBOT_COLOR_BLUE_KEY          ("robot_color_blue");
const std::string ROBOT_COLOR_GREEN_KEY         ("robot_color_green");
const std::string ROBOT_COLOR_GREY_KEY          ("robot_color_grey");
const std::string ROBOT_COLOR_LIGHT_BLUE_KEY    ("robot_color_light_blue");
const std::string ROBOT_COLOR_VIOLET_KEY        ("robot_color_violet");
const std::string FLOW_TAIL_COLOR_KEY           ("flow_tail_color");
const std::string FLOW_HEAD_COLOR_KEY           ("flow_head_color");
const std::string RRT_NODE_COLOR_KEY            ("node_color");
const std::string RRT_EDGE_COLOR_KEY            ("edge_color");
const std::string RRT_START_COLOR_KEY           ("start_color");
const std::string RRT_TARGET_COLOR_KEY          ("target_color");
const std::string RRT_GOAL_REGION_COLOR_KEY     ("goal_region_color");
const std::string RRT_GOAL_PATH_COLOR_KEY       ("goal_path_color");
const std::string ROBOT_MODEL_CONFIG_FILE_KEY   ("robot_model_config_file");
const std::string METRIC_PLANNER_CONFIG_FILE_KEY("metric_planner_config_file");

const std::string RELOCALIZATION_DISPLAY_HEADING("RelocalizationDisplayParameters");
const std::string REQUEST_OUTPUT_CHANNEL_KEY    ("relocalization_request_output_channel");

const std::string SCRIPTING_DISPLAY_HEADING("ScriptingDisplayParameters");
const std::string SCRIPT_HOVER_COLOR_KEY   ("hover_color");
const std::string SCRIPT_SELECT_COLOR_KEY  ("selected_color");
const std::string SCRIPT_FINAL_COLOR_KEY   ("finalized_color");

const std::string DECISION_PLANNER_DISPLAY_HEADING("DecisionPlannerDisplayParameters");
const std::string SEQUENCE_OUTPUT_CHANNEL_KEY     ("target_sequence_output_channel");
const std::string ENTRY_FRAGMENT_COLOR_KEY        ("place_entry_fragment_color");
const std::string EXIT_FRAGMENT_COLOR_KEY         ("place_exit_fragment_color");
const std::string PATH_EXIT_COLOR_KEY             ("path_exit_point_color");

const std::string GOAL_PLANNER_DISPLAY_HEADING("GoalPlannerDisplayParameters");
// Using GLOBAL_LOCATION_COLOR_KEY
const std::string GLOBAL_TARGET_COLOR_KEY           ("global_target_color");
const std::string PLACE_VERTEX_COLOR_KEY            ("place_vertex_color");
const std::string SEGMENT_VERTEX_COLOR_KEY          ("path_segment_vertex_color");
const std::string GRAPH_EDGE_COLOR_KEY              ("graph_edge_color");
const std::string GRAPH_PATH_COLOR_KEY              ("graph_path_color");
const std::string ROUTE_ELEMENT_COLOR_KEY           ("route_element_color");
const std::string VISITED_ELEMENT_COLOR_KEY         ("visited_element_color");
const std::string ACTIVE_ELEMENT_COLOR_KEY          ("active_element_color");
const std::string REMAINING_ELEMENT_COLOR_KEY       ("remaining_element_color");
const std::string ROUTE_COMMAND_OUTPUT_CHANNEL_KEY  ("route_command_output_channel");
const std::string GOAL_TARGET_OUTPUT_CHANNEL_KEY    ("goal_target_output_channel");
const std::string GLOBAL_LOCATION_OUTPUT_CHANNEL_KEY("set_global_location_output_channel");

const std::string VISION_DISPLAY_HEADING("VisionDisplayParameters");
const std::string GROUND_PLANE_COLOR_KEY("ground_plane_boundary_color");

const std::string MAIN_FRAME_HEADING("MainFrameParameters");
const std::string FPS_KEY           ("frames_per_second");


// Helpers for loading the parameters of the various structs
lpm_display_params_t              load_lpm_display_params             (const utils::ConfigFile& config);
local_topo_display_params_t       load_local_topo_display_params      (const utils::ConfigFile& config);
global_topo_display_params_t      load_global_topo_display_params     (const utils::ConfigFile& config);
relocalization_display_params_t   load_relocalization_display_params  (const utils::ConfigFile& config);
scripting_display_params_t        load_scripting_display_params       (const utils::ConfigFile& config);
metric_planner_display_params_t   load_metric_planner_display_params  (const utils::ConfigFile& config);
decision_planner_display_params_t load_decision_planner_display_params(const utils::ConfigFile& config);
goal_planner_display_params_t     load_goal_planner_display_params    (const utils::ConfigFile& config);
vision_display_params_t           load_vision_display_params          (const utils::ConfigFile& config);
main_frame_params_t               load_main_frame_params              (const utils::ConfigFile& config);


ui_params_t load_ui_params(const utils::ConfigFile& config)
{
    ui_params_t params;

    params.lpmParams                  = load_lpm_display_params(config);
    params.localTopoParams            = load_local_topo_display_params(config);
    params.globalTopoParams           = load_global_topo_display_params(config);
    params.relocalizationParams       = load_relocalization_display_params(config);
    params.scriptingParams            = load_scripting_display_params(config);
    params.metricPlannerDisplayParams = load_metric_planner_display_params(config);
    params.decisionPlannerParams      = load_decision_planner_display_params(config);
    params.goalPlannerParams          = load_goal_planner_display_params(config);
    params.visionParams               = load_vision_display_params(config);
    params.mainFrameParams            = load_main_frame_params(config);

    return params;
}


lpm_display_params_t load_lpm_display_params(const utils::ConfigFile& config)
{
    lpm_display_params_t params;

    params.frontLaserColor        = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, FRONT_LASER_COLOR_KEY));
    params.backLaserColor         = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, BACK_LASER_COLOR_KEY));
    params.warningLaserColor      = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, WARNING_LASER_COLOR_KEY));
    params.criticalLaserColor     = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, CRITICAL_LASER_COLOR_KEY));
    params.intensityLaserColor    = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, INTENSITY_LASER_COLOR_KEY));
    params.extractedLinesColor    = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, EXTRACTED_LINES_COLOR_KEY));
    params.occupiedColor          = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, OCCUPIED_COLOR_KEY));
    params.dynamicColor           = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, DYNAMIC_COLOR_KEY));
    params.limitedVisibilityColor = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, LIMITED_VIS_COLOR_KEY));
    params.hazardColor            = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, HAZARD_COLOR_KEY));
    params.quasiStaticColor       = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, QUASI_STATIC_COLOR_KEY));
    params.robotColor             = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, ROBOT_COLOR_KEY));
    params.traceColor             = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, TRACE_COLOR_KEY));
    params.odometryTraceColor     = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, ODOM_TRACE_COLOR_KEY));
    params.particlesColor         = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, PARTICLES_COLOR_KEY));

    params.targetCircleColor = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, CIRCLE_COLOR_KEY));
    params.targetArrowColor  = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, ARROW_COLOR_KEY));
    params.metricPathColor   = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, METRIC_PATH_COLOR_KEY));

    params.reachedTargetCircleColor = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, REACHED_CIRCLE_COLOR_KEY));
    params.reachedTargetArrowColor  = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, REACHED_ARROW_COLOR_KEY));

    params.acquiringTrackingColor = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, ACQUIRING_TARGET_COLOR_KEY));
    params.activeTrackingColor    = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, ACTIVE_TARGET_COLOR_KEY));
    params.inactiveTrackingColor  = GLColor(config.getValueAsString(LPM_DISPLAY_HEADING, INACTIVE_TARGET_COLOR_KEY));

    params.maxTraceLength  = config.getValueAsUInt16(LPM_DISPLAY_HEADING, TRACE_LENGTH_KEY);

    params.messageChannel    = config.getValueAsString(LPM_DISPLAY_HEADING, LOCAL_METRIC_MESSAGE_KEY);
    params.metricPathChannel = config.getValueAsString(LPM_DISPLAY_HEADING, METRIC_PATH_KEY);

    return params;
}


local_topo_display_params_t load_local_topo_display_params(const utils::ConfigFile& config)
{
    local_topo_display_params_t params;

    params.frontierColor     = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, FRONTIER_CELL_COLOR_KEY));
    params.gatewayColor      = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, GATEWAY_CELL_COLOR_KEY));
    params.junctionColor     = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, JUNCTION_COLOR_KEY));
    params.deadEndColor      = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, DEAD_END_COLOR_KEY));
    params.skeletonCellColor = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, SKELETON_CELL_COLOR_KEY));
    params.reducedCellColor  = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, REDUCED_CELL_COLOR_KEY));
    params.sourceColor       = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, SOURCE_CELL_COLOR_KEY));

    params.unknownColor      = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, UNKNOWN_AREA_COLOR_KEY));
    params.pathColor         = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_PATH_COLOR_KEY));
    params.decisionColor     = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_DECISION_COLOR_KEY));
    params.destinationColor  = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_DEST_COLOR_KEY));
    params.pathDecisionColor = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_PATH_DECI_COLOR_KEY));
    params.pathDestColor     = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_PATH_DEST_COLOR_KEY));
    params.destDecisionColor = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_DEST_DECI_COLOR_KEY));

    params.exploredColor        = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, EXPLORED_COLOR_KEY));
    params.gatewayEndpointColor = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, GATEWAY_END_COLOR_KEY));

    params.pathStartColor = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_PATH_START_COLOR_KEY));
    params.pathEndColor   = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, LOCAL_PATH_END_COLOR_KEY));

    params.isovistRayColor      = GLColor(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, ISOVIST_RAY_COLOR_KEY));

    std::vector<std::string> fieldColors = utils::split_into_strings(config.getValueAsString(LOCAL_TOPO_DISPLAY_HEADING, ISOVIST_FIELD_COLORS_KEY), ';');
    for(auto& color : fieldColors)
    {
        params.isovistFieldColors.push_back(GLColor(color));
    }

    return params;
}


global_topo_display_params_t load_global_topo_display_params(const utils::ConfigFile& config)
{
    global_topo_display_params_t params;

    params.pathColor     = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, GLOBAL_PATH_COLOR_KEY));
    params.placeColor    = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, GLOBAL_PLACE_COLOR_KEY));
    params.locationColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, GLOBAL_LOCATION_COLOR_KEY));
    params.frontierColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, GLOBAL_FRONTIER_COLOR_KEY));

    params.nodeColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_NODE_COLOR_KEY));
    params.edgeColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_EDGE_COLOR_KEY));

    params.bestNodeColor     = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_BEST_COLOR_KEY));
    params.hoverNodeColor    = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_HOVER_COLOR_KEY));
    params.selectedMapAColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_SEL_A_COLOR_KEY));
    params.selectedMapBColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_SEL_B_COLOR_KEY));
    params.selectedMapCColor = GLColor(config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, HYP_TREE_SEL_C_COLOR_KEY));

    params.correctMapOutputChannel = config.getValueAsString(GLOBAL_TOPO_DISPLAY_HEADING, CORRECT_MAP_CHANNEL_KEY);

    return params;
}


relocalization_display_params_t load_relocalization_display_params(const utils::ConfigFile& config)
{
    relocalization_display_params_t params;

    params.requestChannel = config.getValueAsString(RELOCALIZATION_DISPLAY_HEADING, REQUEST_OUTPUT_CHANNEL_KEY);

    return params;
}


scripting_display_params_t load_scripting_display_params(const utils::ConfigFile& config)
{
    scripting_display_params_t params;

    params.hoverColor     = GLColor(config.getValueAsString(SCRIPTING_DISPLAY_HEADING, SCRIPT_HOVER_COLOR_KEY));
    params.selectedColor  = GLColor(config.getValueAsString(SCRIPTING_DISPLAY_HEADING, SCRIPT_SELECT_COLOR_KEY));
    params.finalizedColor = GLColor(config.getValueAsString(SCRIPTING_DISPLAY_HEADING, SCRIPT_FINAL_COLOR_KEY));

    return params;
}


metric_planner_display_params_t load_metric_planner_display_params(const utils::ConfigFile& config)
{
    metric_planner_display_params_t params;

    params.trajectoryColorRed   = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, TRAJECTORY_COLOR_RED_KEY));
    params.trajectoryColorBlue  = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, TRAJECTORY_COLOR_BLUE_KEY));
    params.trajectoryColorGreen = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, TRAJECTORY_COLOR_GREEN_KEY));

    params.robotColorRed       = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_RED_KEY));
    params.robotColorBlue      = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_BLUE_KEY));
    params.robotColorGreen     = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_GREEN_KEY));
    params.robotColorGrey      = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_GREY_KEY));
    params.robotColorLightBlue = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_LIGHT_BLUE_KEY));
    params.robotColorViolet    = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_COLOR_VIOLET_KEY));

    params.flowTailColor = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, FLOW_TAIL_COLOR_KEY));
    params.flowHeadColor = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, FLOW_HEAD_COLOR_KEY));

    params.nodeColor       = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_NODE_COLOR_KEY));
    params.edgeColor       = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_EDGE_COLOR_KEY));
    params.startColor      = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_START_COLOR_KEY));
    params.targetColor     = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_TARGET_COLOR_KEY));
    params.goalRegionColor = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_GOAL_REGION_COLOR_KEY));
    params.goalPathColor   = GLColor(config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, RRT_GOAL_PATH_COLOR_KEY));

    params.collisionModelConfig = config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, ROBOT_MODEL_CONFIG_FILE_KEY);
    params.mpepcConfig = config.getValueAsString(METRIC_PLANNER_DISPLAY_HEADING, METRIC_PLANNER_CONFIG_FILE_KEY);

    return params;
}


decision_planner_display_params_t load_decision_planner_display_params(const utils::ConfigFile& config)
{
    decision_planner_display_params_t params;

    params.targetSequenceChannel   = config.getValueAsString(DECISION_PLANNER_DISPLAY_HEADING, SEQUENCE_OUTPUT_CHANNEL_KEY);
    params.placeEntryFragmentColor = GLColor(config.getValueAsString(DECISION_PLANNER_DISPLAY_HEADING, ENTRY_FRAGMENT_COLOR_KEY));
    params.placeExitFragmentColor  = GLColor(config.getValueAsString(DECISION_PLANNER_DISPLAY_HEADING, EXIT_FRAGMENT_COLOR_KEY));
    params.pathExitPointColor      = GLColor(config.getValueAsString(DECISION_PLANNER_DISPLAY_HEADING, PATH_EXIT_COLOR_KEY));

    return params;
}


goal_planner_display_params_t load_goal_planner_display_params(const utils::ConfigFile& config)
{
    goal_planner_display_params_t params;

    params.globalLocationColor = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GLOBAL_LOCATION_COLOR_KEY));
    params.globalTargetColor   = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GLOBAL_TARGET_COLOR_KEY));

    params.placeVertexColor       = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, PLACE_VERTEX_COLOR_KEY));
    params.pathSegmentVertexColor = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, SEGMENT_VERTEX_COLOR_KEY));
    params.edgeColor              = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GRAPH_EDGE_COLOR_KEY));
    params.graphPathColor         = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GRAPH_PATH_COLOR_KEY));

    params.routeElementColor     = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, ROUTE_ELEMENT_COLOR_KEY));
    params.visitedElementColor   = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, VISITED_ELEMENT_COLOR_KEY));
    params.activeElementColor    = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, ACTIVE_ELEMENT_COLOR_KEY));
    params.remainingElementColor = GLColor(config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, REMAINING_ELEMENT_COLOR_KEY));

    params.routeCommandOutputChannel      = config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, ROUTE_COMMAND_OUTPUT_CHANNEL_KEY);
    params.goalTargetOutputChannel        = config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GOAL_TARGET_OUTPUT_CHANNEL_KEY);
    params.setGlobalLocationOutputChannel = config.getValueAsString(GOAL_PLANNER_DISPLAY_HEADING, GLOBAL_LOCATION_OUTPUT_CHANNEL_KEY);

    return params;
}


vision_display_params_t load_vision_display_params(const utils::ConfigFile& config)
{
    vision_display_params_t params;

    params.groundPlaneColor = GLColor(config.getValueAsString(VISION_DISPLAY_HEADING, GROUND_PLANE_COLOR_KEY));

    return params;
}


main_frame_params_t load_main_frame_params(const utils::ConfigFile& config)
{
    main_frame_params_t params;

    params.framesPerSecond = config.getValueAsUInt8(MAIN_FRAME_HEADING, FPS_KEY);

    return params;
}

} // namespace ui
} // namespace vulcan
