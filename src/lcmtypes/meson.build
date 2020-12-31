lcmtypes = [
    'commands/command_result.lcm',
    'commands/direct_control_command.lcm',
    'commands/load_lpm_command.lcm',
    'commands/save_lpm_command.lcm',
    'commands/turn_around_command.lcm',
    'commands/turn_command.lcm',
    'common/circle_t.lcm',
    'common/gaussian_distribution_t.lcm',
    'common/matrix_t.lcm',
    'common/point_array_t.lcm',
    'common/point_float_t.lcm',
    'common/point_int16_t.lcm',
    'common/point_t.lcm',
    'common/rectangle_t.lcm',
    'common/vector_t.lcm',
    'legacy/robot_pose_old_t.lcm',
    'legacy/commanded_velocity_t.lcm',
    'legacy/commanded_joystick_t.lcm',
    'legacy/laser_t.lcm',
    'legacy/laser_old_t.lcm',
    'legacy/laser_with_intensity_t.lcm',
    'legacy/encoder_data_t.lcm',
    'legacy/joystick_command_t.lcm',
    'legacy/velocity_command_t.lcm',
    'legacy/odometry_t.lcm',
    'legacy/imu_t.lcm',
    'legacy/laser_3dof_t.lcm',
    'mpepc/metric_pose_target_status_t.lcm',
    'mpepc/metric_pose_target_t.lcm',
    'state/motion_state_t.lcm',
    'state/drive_wheel_t.lcm',
    'state/pose_distribution_t.lcm',
    'state/robot_acceleration_t.lcm',
    'state/elevator_t.lcm',
    'state/robot_pose_t.lcm',
    'state/robot_velocity_t.lcm',
]

lcmgen = find_program('lcm-gen')

lcm_gen = generator(lcmgen,
   output : [
    'vulcan_lcm_@BASENAME@.c', 
    'vulcan_lcm_@BASENAME@.h', 
    '@BASENAME@.hpp'],
   arguments : [
    '-c', 
    '--c-cpath=@BUILD_DIR@', 
    '--c-hpath=@BUILD_DIR@', 
    '--lazy',
    '-x',
    '--cpp-hpath=@BUILD_DIR@',
    '-j', 
    '--jpath=@BUILD_DIR@/java/',
    '@INPUT@'])

lcm_src = lcm_gen.process(lcmtypes)

vulcan_to_lcm_src = [
    'common/gaussian_distribution_t.cpp',
    'common/vector_t.cpp',
    'common/matrix_t.cpp',
    'legacy/imu_t.cpp',
    'legacy/velocity_command_t.cpp',
    'legacy/laser_t.cpp',
    'legacy/commanded_velocity_t.cpp',
    'legacy/encoder_data_t.cpp',
    'legacy/commanded_joystick_t.cpp',
    'legacy/odometry_t.cpp',
    'legacy/joystick_command_t.cpp',
    'message_helpers.cpp',
    'state/elevator_t.cpp',
    'state/robot_pose_t.cpp',
    'state/robot_velocity_t.cpp',
    'state/pose_distribution_t.cpp',
    'state/motion_state_t.cpp',
]

lcm_lib = static_library(
    'vulcan_lcm',
    [lcm_src, vulcan_to_lcm_src],
    include_directories : vulcan_inc,
    dependencies: [lcm_dep]
)
