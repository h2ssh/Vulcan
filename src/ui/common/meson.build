ui_common_src = [
    'color_generator.cpp',
    'color_interpolator.cpp',
    'default_colors.cpp',
    'gaussian_creator.cpp',
    'gl_arrays_helpers.cpp',
    'gl_camera_controller.cpp',
    'gl_camera.cpp',
    'gl_movie_maker.cpp',
    'gl_shapes.cpp',
    'gl_texture_helpers.cpp',
    'gl_utilities.cpp',
    'grid_cell_selector.cpp',
    'metric_path_creator.cpp',
    'shape_creators.cpp',
    'ui_color.cpp',
    'ui_main_frame.cpp',
    'ui_params.cpp',
]

ui_common_lib = static_library(
    'ui_common_lib',
    ui_common_src,
    include_directories : [ vulcan_inc ],
    dependencies : [ ui_dep ],
)
