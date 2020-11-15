############################## Laser producer ###################################

urg_dep = dependency('liburg')

laser_producer = executable(
    'laser_scan_producer',
    ['hokuyo_urg_laser.cpp', 'laser_scan_producer.cpp'],
    include_directories : vulcan_inc,
    dependencies : urg_dep,
    link_with : [core_lib, system_lib, lcm_lib, utils_lib],
    install : true
)

############################## IMU producer ###################################

imu_src = ['microstrain_3dmgx2.cpp',
           'microstrain_3dmgx3.cpp',]

imu_producer = executable(
    'imu_data_producer',
    [ imu_src, 'imu_data_producer.cpp'],
    include_directories : vulcan_inc,
    link_with : [core_lib, system_lib, lcm_lib, utils_lib],
    install : true
)

############################## Odometry producer ###################################

phidgets_dep = dependency('libphidget21')

odometry_src = ['wheel_encoders.cpp',
                'odometry_estimator.cpp',
                'phidget_encoder_board.cpp',
                'remote_encoders.cpp',
                'wheel_encoders_params.cpp']

odometry_producer = executable(
    'odometry_producer',
    [odometry_src, 'odometry_producer.cpp'],
    include_directories : vulcan_inc,
    dependencies : phidgets_dep,
    link_with : [core_lib, system_lib, lcm_lib, utils_lib],
    install : true
)

################################ Sensors library ##################################

sensor_lib_src = ['wheel_encoders.cpp',
                  'wheel_encoders_params.cpp',
                  'sensor_log.cpp']

sensor_lib = static_library(
    'vulcan_sensors',
    sensor_lib_src,
    include_directories : vulcan_inc,
    link_with : [core_lib, lcm_lib, utils_lib, system_lib]
)
