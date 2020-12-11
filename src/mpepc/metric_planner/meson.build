metric_planner_module_src = [
    'director.cpp',
    'main.cpp',
    'progress_checker.cpp',
]


metric_planner = executable(
    'metric_planner',
    metric_planner_module_src,
    include_directories : vulcan_inc,
    link_with : [ mpepc_data_lib, mpepc_func_lib, system_lib, utils_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
