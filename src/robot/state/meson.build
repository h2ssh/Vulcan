state_estimator_src = [
    'director.cpp',
    'drive_wheel_estimator.cpp',
    'elevator_monitor.cpp',
    'main.cpp',
    'motion_state_estimator.cpp',
    'motion_state_input.cpp',
    'params.cpp',
    'pose_estimator.cpp',
    'state_estimator.cpp',
    'velocity_estimator.cpp',
]

state_estimator = executable(
    'state_estimator',
    state_estimator_src,
    include_directories : vulcan_inc,
    link_with : [ core_lib, lcm_lib, robot_model_lib, system_lib, utils_lib ],
    dependencies : [ lcm_dep, boost_dep, thread_dep ],
    install : true,
)
