hssh_metrical_src = [
    'data_queue.cpp',
    'glass_map.cpp',
    'occupancy_grid.cpp',
]

hssh_metrical_localization_src = [
    'localization/adaptive_particle_filter.cpp',
    'localization/discrete_beam_model.cpp',
    'localization/gaussian_observation_model.cpp',
    'localization/grid_based_observation_models.cpp',
    'localization/kld_sample_size_calculator.cpp',
    'localization/known_pose_localizer.cpp',
    'localization/localizer.cpp',
    'localization/monte_carlo.cpp',
    'localization/motion_model.cpp',
    'localization/motion_model_distribution.cpp',
    'localization/observation_model.cpp',
    'localization/params.cpp',
    'localization/particle_filter.cpp',
    'localization/particle_filter_utils.cpp',
    'localization/particle_sampler.cpp',
    'localization/sample_set_distribution_calculator.cpp',
    'localization/scan_weighting.cpp',
    'localization/vanilla_particle_filter.cpp',
]

hssh_metrical_mapping_src = [
    'mapping/glass_map_builder.cpp',
    'mapping/glass_map_utils.cpp',
    'mapping/glass_walls.cpp',
    'mapping/laser_scan_rasterizer.cpp',
    'mapping/lpm_builder.cpp',
    'mapping/mapper.cpp',
    'mapping/mapping_params.cpp',
]

hssh_metrical_relocalization_src = [
    'relocalization/filter_initializer_impl.cpp',
    'relocalization/metric_relocalizer.cpp',
    'relocalization/params.cpp',
    'relocalization/particle_sampling.cpp',
    'relocalization/scan_matching_initializer.cpp',
]

hssh_metrical_lib = static_library(
    'hssh_metrical_lib',
    [
      hssh_metrical_src,
      hssh_metrical_localization_src,
      hssh_metrical_mapping_src,
      hssh_metrical_relocalization_src
    ],
    include_directories : vulcan_inc,
    link_with : [ core_lib, math_lib, laser_lib, utils_lib ],
)
