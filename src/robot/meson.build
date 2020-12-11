robot_model_src = [
    'model/differential_motors_plant.cpp',
    'model/differential_torque_plant.cpp',
    'model/kinematic_robot_plant.cpp',
    'model/motor_model.cpp',
    'model/odometry_models.cpp',
    'model/params.cpp',
    'model/pd_robot_plant.cpp',
    'model/robot_plant_model.cpp',
]


robot_model_lib = static_library(
    'robot_model_lib',
    robot_model_src,
    include_directories : [ vulcan_inc ],
    link_with : [ utils_lib ],
)

subdir('state')
