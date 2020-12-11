mpepc_data_src = [
    'cost/obstacle_cost.cpp',
    'cost/quasi_static_cost.cpp',
    'cost/social_cost.cpp',
    'evaluation/mpepc_log.cpp',
    'evaluation/path_summary.cpp',
    'evaluation/utils.cpp',
    'grid/navigation_grid.cpp',
    'grid/navigation_grid_builder.cpp',
    'grid/navigation_grid_utils.cpp',
    'grid/obstacle_distance_grid.cpp',
    'grid/obstacle_distance_grid_builder.cpp',
    'grid/params.cpp',
    'grid/visibility_analysis.cpp',
    'manifold/navigation.cpp',
    'manifold/rotate.cpp',
    'manifold/wait.cpp',
    'metric_planner/script/script.cpp',
    'metric_planner/script/target_set.cpp',
    'metric_planner/task/navigation.cpp',
    'metric_planner/task/params.cpp',
    'metric_planner/task/rotate.cpp',
    'metric_planner/task/wait.cpp',
    'social/social_norm_utils.cpp',
    'social/topo_agent.cpp',
    'social/topo_situation.cpp',
    'trajectory/robot_trajectory_info.cpp',
]

mpepc_func_src = [
    'collision/collision_detection.cpp',
    'collision/robot_collision_model.cpp',
    'control/joystick_control_law.cpp',
    'control/kinematic_control_law.cpp',
    'control/params.cpp',
    'math/unicycle_chart.cpp',
    'math/unicycle_lyapunov_distance.cpp',
    'math/unicycle_nonholonomic_distance.cpp',
    'metric_planner/params.cpp',
    'rrt/unicycle_lyapunov_steering.cpp',
    'rrt/unicycle_pose_sampler.cpp',
    'rrt/unicycle_rrt_star.cpp',
    'rrt/unicycle_rrt_tree.cpp',
    'simulator/dynamic_object_filter.cpp',
    'simulator/dynamic_object_simulator.cpp',
    'simulator/params.cpp',
    'simulator/pedestrian_generator.cpp',
    'simulator/robot_simulator.cpp',
    'trajectory/mpepc_optimizer.cpp',
    'trajectory/mpepc_pose_follower.cpp',
    'trajectory/params.cpp',
    'trajectory/trajectory_evaluator.cpp',
    'trajectory/trajectory_planner.cpp',
]


mpepc_data_lib = static_library(
    'mpepc_data_lib',
    mpepc_data_src,
    include_directories : vulcan_inc,
    link_with : [ local_metric_data_lib, local_topo_debug_lib, robot_model_lib, tracker_debug_lib, utils_lib ],
)

mpepc_func_lib = static_library(
    'mpepc_func_lib',
    mpepc_func_src,
    include_directories : vulcan_inc,
    link_with : [ mpepc_data_lib, robot_model_lib, utils_lib ],
    dependencies : [ nlopt_dep ],
)


subdir('metric_planner')
subdir('motion_controller')
