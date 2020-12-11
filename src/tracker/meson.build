tracker_data_src = [
    'boundaries/circle.cpp',
    'boundaries/rectangle.cpp',
    'goals/ballistic_estimator.cpp',
    'goals/fixed_estimator.cpp',
    'goals/gateway_estimator.cpp',
    'goals/goal_estimator.cpp',
    'goals/goal_estimator_factory.cpp',
    'motions/classifier.cpp',
    'motions/fixed_endpoint.cpp',
    'motions/local_trajectory_regression.cpp',
    'motions/motion_tracker.cpp',
    'motions/stationary.cpp',
    'motions/steady.cpp',
    'motions/striding.cpp',
    'objects/endpoint_model.cpp',
    'objects/fixed_object.cpp',
    'objects/fixed_object_model.cpp',
    'objects/leg.cpp',
    'objects/object_factory.cpp',
    'objects/person.cpp',
    'objects/pivoting_object.cpp',
    'objects/rigid.cpp',
    'objects/sliding_object.cpp',
    'objects/unclassified.cpp',
    'utils/endpoints.cpp',
    'laser_object.cpp',
    'object_boundary.cpp',
    'object_motion.cpp',
]

tracker_debug_src = [
    'evaluation/intention_evaluator.cpp',
]

tracker_func_src = [
    'tracking/data_association.cpp',
    'tracking/max_likelihood_association.cpp',
    'tracking/nearest_neighbor_association.cpp',
    'tracking/object.cpp',
    'tracking/object_factory.cpp',
    'tracking/object_set.cpp',
    'goal_predictor.cpp',
    'object_detector.cpp',
    'object_tracker.cpp',
]

tracker_module_src = [
    'director.cpp',
    'main.cpp'
]

tracker_data_lib = static_library(
    'tracker_data_lib',
    tracker_data_src,
    include_directories : vulcan_inc,
)

tracker_debug_lib = static_library(
    'tracker_debug_lib',
    tracker_debug_src,
    include_directories : vulcan_inc,
    link_with : [ tracker_data_lib ],
)

tracker_func_lib = static_library(
    'tracker_func_lib',
    tracker_func_src,
    include_directories : vulcan_inc,
    link_with : [ tracker_data_lib, tracker_debug_lib, local_topo_data_lib ],
)

object_tracker = executable(
    'object_tracker',
    tracker_module_src,
    include_directories : vulcan_inc,
    link_with : [ lcm_lib, system_lib, tracker_func_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep ],
    install : true,
)
