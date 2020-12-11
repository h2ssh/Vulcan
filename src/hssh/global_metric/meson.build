global_metric_data_src = [
    'map.cpp',
    'pose.cpp',
]

global_metric_module_src = [
    'director.cpp',
    'main.cpp',
]

global_metric_data_lib = static_library(
    'global_metric_data_lib',
    global_metric_data_src,
    include_directories : vulcan_inc,
    link_with : [ hssh_metrical_lib, local_metric_data_lib ],
)

global_metric_hssh = executable(
    'global_metric_hssh',
    global_metric_module_src,
    include_directories : vulcan_inc,
    link_with : [ system_lib, lcm_lib, global_metric_data_lib ],
    dependencies : [ thread_dep, lcm_dep, boost_dep ],
    install : true,
)
