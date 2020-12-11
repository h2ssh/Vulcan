planner_utils_src = [
    'utils/local_area_tasks.cpp',
]

planner_utils_lib = static_library(
    'planner_utils_lib',
    planner_utils_src,
    include_directories : [ vulcan_inc ],
    link_with : [ local_topo_data_lib, mpepc_data_lib ],
)

subdir('control')
subdir('interface')
subdir('exploration')
