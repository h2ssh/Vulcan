laser_src = [
    'laser_io.cpp',
    'correlative_scan_matcher.cpp',
    'scan_likelihood_grid.cpp',
    'scan_matcher_params.cpp',
    'laser_line_extractor.cpp',
    'line_extraction.cpp',
    'line_extractor_params.cpp',
]

laser_lib = static_library(
    'vulcan_laser',
    laser_src,
    include_directories : vulcan_inc,
    link_with : [core_lib],
)


estimate_laser_noise = executable(
    'estimate_laser_noise',
    'estimate_laser_noise.cpp',
    include_directories : vulcan_inc,
    link_with : [laser_lib, sensor_lib],
    dependencies : [boost_dep],
    install : true,
)

subdir('test')
