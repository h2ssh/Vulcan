local_metric_data_src = [
    'commands/glass_evaluation.cpp',
    'commands/relocalize_in_lpm.cpp',
    'commands/rotate_lpm.cpp',
    'commands/save_metric_map.cpp',
    'commands/set_map.cpp',
    'commands/set_slam_mode.cpp',
    'commands/toggle_glass_mapping.cpp',
    'commands/truncate_lpm.cpp',
    'lpm.cpp',
    'lpm_io.cpp',
    'map_rectification.cpp',
]

local_multifloor_src = [
    'multifloor/elevator.cpp',
    'multifloor/floor.cpp',
    'multifloor/mapper.cpp',
    'multifloor/multi_floor_map_io.cpp',
    'multifloor/params.cpp',
]

local_metric_module_src = [
    'director.cpp',
    'main.cpp',
    'params.cpp',
]


local_metric_data_lib = static_library(
    'local_metric_data_lib',
    local_metric_data_src,
    include_directories : vulcan_inc,
    link_with : [ hssh_metrical_lib, vision_lib ],
)

local_metric_hssh = executable(
    'local_metric_hssh',
    [
        local_multifloor_src,
        local_metric_module_src,
    ],
    include_directories : vulcan_inc,
    link_with : [ system_lib, lcm_lib, local_metric_data_lib ],
    dependencies : [ thread_dep, lcm_dep, boost_dep ],
    install : true,
)

save_random_lpms = executable(
    'save_random_lpms',
    'save_random_lpms.cpp',
    include_directories : vulcan_inc,
    link_with : [ system_lib, local_metric_data_lib ],
    dependencies : [ lcm_dep ],
    install : true,
)
