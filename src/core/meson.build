core_src = [
    'conversions.cpp',
    'drive_wheel.cpp',
    'image.cpp',
    'imu_data.cpp',
    'laser_scan.cpp',
    'multivariate_gaussian.cpp',
    'odometry.cpp',
    'pose.cpp',
    'pose_distribution.cpp',
    'velocity.cpp',
]

core_lib = static_library(
    'vulcan_core',
    core_src,
    include_directories : vulcan_inc,
    dependencies : [arma_dep],
)
