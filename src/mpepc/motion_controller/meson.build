motion_controller_src = [
    'controller/controller_chain.cpp',
    'controller/motion_controller.cpp',
    'controller/motion_target_following_controller.cpp',
    'controller/path_following_controller.cpp',
    'waypoint_follower/graceful_motion_control_law.cpp',
    'waypoint_follower/params.cpp',
    'waypoint_follower/waypoint_decider.cpp',
    'waypoint_follower/waypoint_follower.cpp',
]

motion_controller_module_src = [
    'director.cpp',
    'main.cpp',
    'params.cpp',
]


motion_controller = executable(
    'motion_controller',
    [ motion_controller_src, motion_controller_module_src ],
    include_directories : vulcan_inc,
    link_with : [ lcm_lib, mpepc_func_lib, robot_model_lib, system_lib, utils_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
