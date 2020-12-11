local_topo_data_src = [
    'affordances/exploration.cpp',
    'affordances/move_along.cpp',
    'affordances/transition.cpp',
    'areas/decision_point.cpp',
    'areas/destination.cpp',
    'areas/path_segment.cpp',
    'areas/place.cpp',
    'area_detection/voronoi/search.cpp',
    'area_detection/voronoi/voronoi_utils.cpp',
    'area_detection/gateways/gateway_utils.cpp',
    'evaluation/heat_map.cpp',
    'evaluation/topological_edit_distance.cpp',
    'events/area_transition.cpp',
    'events/turn_around.cpp',
    'area.cpp',
    'area_extent.cpp',
    'gateway.cpp',
    'lambda.cpp',
    'local_topo_graph.cpp',
    'local_topo_map.cpp',
    'local_topo_route.cpp',
    'metric_place_io.cpp',
    'small_scale_star.cpp',
    'voronoi_skeleton_grid.cpp',
]

local_topo_debug_src = [
    'area_detection/labeling/area_proposal.cpp',
    'area_detection/labeling/hypothesis.cpp',
    'area_detection/labeling/hypothesis_features.cpp',
    'area_detection/labeling/hypothesis_type.cpp',
    'area_detection/labeling/boundary.cpp',
    'area_detection/labeling/boundary_classifier.cpp',
    'area_detection/labeling/hypothesis_classifier.cpp',
    'area_detection/labeling/path_endpoints.cpp',
    'area_detection/labeling/path_similarity_star_builder.cpp',
    'area_detection/local_topo_isovist_field.cpp',
    'area_detection/gateways/isovist_gradients.cpp',
    'area_detection/gateways/isovist_maxima.cpp',
    'area_detection/gateways/feature_extraction.cpp',
    'area_detection/gateways/gateway_classifier.cpp',
    'area_detection/labeling/area_graph.cpp',
    'area_detection/labeling/loops_and_trees.cpp',
    'area_detection/voronoi/voronoi_edges.cpp',
    'evaluation/stability_analyzer.cpp',
    'evaluation/stability_log.cpp',
    'training/labeled_area_data.cpp',
    'training/labeled_boundary_data.cpp',
    'training/labeled_gateway_data.cpp',
    'training/gateway_classifier_test.cpp',
    'training/gateway_errors.cpp',
    'training/hypothesis_classifier_test.cpp',
    'params.cpp',
]

local_topo_func_src = [
    'area_detection/gateways/classifier_based_generator.cpp',
    'area_detection/gateways/feature_extraction.cpp',
    'area_detection/gateways/filters.cpp',
    'area_detection/gateways/gateway_classifier.cpp',
    'area_detection/gateways/gateway_locator.cpp',
    'area_detection/gateways/gateway_utils.cpp',
    'area_detection/gateways/generator.cpp',
    'area_detection/gateways/isovist_gradients.cpp',
    'area_detection/gateways/isovist_maxima.cpp',
    'area_detection/gateways/isovist_orientation_gateway_generator.cpp',
    'area_detection/gateways/isovist_voronoi_gateway_generator.cpp',
    'area_detection/labeling/area_creator.cpp',
    'area_detection/labeling/parser.cpp',
    'area_detection/labeling/hypothesis_association.cpp',
    'area_detection/labeling/csp_solution.cpp',
    'area_detection/labeling/max_likelihood_csp.cpp',
    'area_detection/labeling/alignment_constraint.cpp',
    'area_detection/labeling/mcmc_sampling.cpp',
    'area_detection/labeling/hypothesis_constraints.cpp',
    'area_detection/labeling/hypothesis_graph.cpp',
    'area_detection/labeling/beam_star_builder.cpp',
    'area_detection/labeling/small_scale_star_builder.cpp',
    'area_detection/labeling/belief_prop.cpp',
    'area_detection/labeling/factor.cpp',
    'area_detection/labeling/factor_edge.cpp',
    'area_detection/labeling/factor_graph.cpp',
    'area_detection/labeling/factor_graph_utils.cpp',
    'area_detection/labeling/hypothesis_factor_graph.cpp',
    'area_detection/labeling/variable.cpp',
    'area_detection/voronoi/all_paths_graph_reducer.cpp',
    'area_detection/voronoi/brushfire_skeleton_builder.cpp',
    'area_detection/voronoi/frontiers_set.cpp',
    'area_detection/voronoi/island_detector.cpp',
    'area_detection/voronoi/voronoi_utils.cpp',
    'area_detection/voronoi/skeleton_builder.cpp',
    'area_detection/voronoi/skeleton_pruner.cpp',
    'area_detection/voronoi/skeleton_graph_extractor.cpp',
    'area_detection/voronoi/skeleton_graph_reducer.cpp',
    'area_detection/voronoi/skeleton_graph_rasterization.cpp',
    'area_detection/voronoi_graph_builder.cpp',
    'area_detection/area_classifier.cpp',
    'area_detection/affordance_labeling_classifier.cpp',
    'area_detection/stored_map_classifier.cpp',
    'boundary/fixed.cpp',
    'boundary/shrink_and_stretch.cpp',
    'boundary/small_scale_space_boundary.cpp',
    'event_detection/area_event_detector.cpp',
    'event_detection/area_transition_detector.cpp',
    'event_detection/path_direction_detector.cpp',
    'area_detector.cpp',
    'event_detector.cpp',
    'localizer.cpp',
]

local_topo_training_src = [
    'training/area_labels.cpp',
    'training/local_topo_area_editor.cpp',
]

local_topo_module_src = [
    'director.cpp',
    'main.cpp',
]


local_topo_data_lib = static_library(
    'local_topo_data_lib',
    local_topo_data_src,
    include_directories : vulcan_inc,
    link_with : [ local_metric_data_lib ],
)

local_topo_debug_lib = static_library(
    'local_topo_debug_lib',
    local_topo_debug_src,
    include_directories : vulcan_inc,
    link_with : [ local_topo_data_lib, global_metric_data_lib ],
    dependencies : [ thread_dep ],
)

local_topo_func_lib = static_library(
    'local_topo_func_lib',
    local_topo_func_src,
    include_directories : vulcan_inc,
    link_with : [ local_topo_debug_lib ],
)

local_topo_training_lib = static_library(
    'local_topo_training_lib',
    local_topo_training_src,
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
)

local_topo_hssh = executable(
    'local_topo_hssh',
    local_topo_module_src,
    include_directories : vulcan_inc,
    link_with : [ system_lib, lcm_lib, local_topo_func_lib ],
    dependencies : [ thread_dep, lcm_dep, boost_dep ],
    install : true,
)

train_classifiers = executable(
    'train_area_classifiers',
    'training/train_area_classifiers.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_training_lib ],
    dependencies : [ thread_dep, boost_dep ],
    install : true,
)

fix_ltm = executable(
    'fix_ltm',
    'training/fix_ltm.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_training_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

eval_labels = executable(
    'evaluate_labels',
    'evaluation/evaluate_labels.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

summarize_results = executable(
    'summarize_label_results',
    'evaluation/summarize_label_results.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

comparison_plot = executable(
    'create_comparison_plots',
    'evaluation/create_comparison_plots.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

generate_heat_maps = executable(
    'generate_label_heat_maps',
    'evaluation/generate_label_heat_maps.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

gen_events = executable(
    'generate_event_sequence',
    'evaluation/generate_event_sequence.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)

ltm_to_ppm = executable(
    'ltm_to_ppm',
    'evaluation/ltm_to_ppm.cpp',
    include_directories : vulcan_inc,
    link_with : [ local_topo_func_lib ],
    dependencies : [ boost_dep ],
    install : true,
)
