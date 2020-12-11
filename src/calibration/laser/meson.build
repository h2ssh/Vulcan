laser_calibration_lib_src = [
    'tilt_calibration.cpp',
]

laser_calibration_lib = static_library(
    'laser_calibration_lib',
    laser_calibration_lib_src,
    include_directories : vulcan_inc,
    link_with : [ core_lib, math_lib ],
    dependencies : [ boost_dep ]
)
