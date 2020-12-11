exploration_data_src = [
    'local_topo/exploration_map.cpp',
    'local_topo/target_impl.cpp',
]

exploration_func_src = [
    'local_topo/local_topo_explorer.cpp',
    'map_explorer.cpp',
]

exploration_module_src = [
    'director.cpp',
    'main.cpp',
]


exploration_data_lib = static_library(
    'exploration_data_lib',
    exploration_data_src,
    include_directories : [ vulcan_inc ],
    link_with : [ local_topo_data_lib, math_lib, mpepc_data_lib, planner_utils_lib ],
)

exploration_func_lib = static_library(
    'exploration_func_lib',
    exploration_func_src,
    include_directories : [ vulcan_inc ],
    link_with : [ exploration_data_lib ],
)

map_exploration = executable(
    'map_exploration',
    exploration_module_src,
    include_directories : [ vulcan_inc ],
    link_with : [ exploration_func_lib, system_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
