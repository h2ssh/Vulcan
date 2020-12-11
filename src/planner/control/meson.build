control_planner_data_src = [
    'tasks/absolute_motion.cpp',
    'tasks/navigation.cpp',
    'tasks/pose.cpp',
    'constriction.cpp',
    'planner.cpp',
    'task.cpp',
]

control_planner_module_src = [
    'director.cpp',
    'main.cpp',
]


control_planner_data_lib = static_library(
    'control_planner_data_lib',
    control_planner_data_src,
    include_directories : vulcan_inc,
)

control_planner = executable(
    'control_planner',
    control_planner_module_src,
    include_directories : vulcan_inc,
    link_with : [ control_planner_data_lib, mpepc_data_lib, system_lib, utils_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
