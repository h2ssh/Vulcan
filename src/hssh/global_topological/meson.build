global_topo_data_src = [
    'utils/local_to_global.cpp',
    'utils/visit.cpp',
    'area.cpp',
    'chi.cpp',
    'global_place.cpp',
    'global_location.cpp',
    'global_path.cpp',
    'global_path_segment.cpp',
    'graph.cpp',
    'topological_map.cpp',
    'transition.cpp',
    'transition_cycle.cpp',
    'transition_sequence.cpp',
]

global_topo_debug_src = [
    'mapping/probability_heuristics.cpp',
    'mapping/tree_of_maps.cpp',
    'utils/metric_map_cache.cpp',
]

global_topo_func_src = [
    'commands/save_topo_slam_data.cpp',
    'localization/localizer.cpp',
    'localization/location_distribution.cpp',
    'localization/utils.cpp',
    'mapping/generator_queue.cpp',
    'mapping/generator_queue_impl.cpp',
    'mapping/hypothesis_generator.cpp',
    'mapping/hypothesis_generator_factory_impl.cpp',
    'mapping/lazy_generator.cpp',
    'mapping/lev_mar_optimizer.cpp',
    'mapping/loop_closures.cpp',
    'mapping/map_optimizer.cpp',
    'mapping/probability_evaluator.cpp',
    'mapping/likelihood_evaluator.cpp',
    'mapping/likelihood_evaluator_impl.cpp',
    'mapping/prior_evaluator.cpp',
    'mapping/prior_evaluator_impl.cpp',
    'utils/event_log_runner.cpp',
    'utils/visit_sequence.cpp',
    'topo_slam.cpp',
]

global_topo_module_src = [
    'utils/global_topo_data_queue.cpp',
    'director.cpp',
    'main.cpp',
    'params.cpp',
]


global_topo_data_lib  = static_library(
    'global_topo_data_lib',
    global_topo_data_src,
    include_directories : vulcan_inc,
    link_with : [ hssh_utils_lib, local_topo_data_lib ],
)

global_topo_debug_lib = static_library(
    'global_topo_debug_lib',
    global_topo_debug_src,
    include_directories : vulcan_inc,
    link_with : [ global_topo_data_lib ]
)

global_topo_func_lib = static_library(
    'global_topo_hssh_func',
    global_topo_func_src,
    include_directories : vulcan_inc,
    link_with : [ global_topo_debug_lib, local_topo_debug_lib ],
    dependencies : [ lapack_dep, blas_dep, levmar_dep ],
)

global_topo_hssh = executable(
    'global_topo_hssh',
    global_topo_module_src,
    include_directories : vulcan_inc,
    link_with : [ system_lib, global_topo_func_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
