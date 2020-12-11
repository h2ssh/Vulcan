planner_interface_data_src = [
    'decision.cpp',
    'decision_action.cpp',
    'decision_interface.cpp',
    'goal.cpp',
    'goal_action.cpp',
    'goal_interface.cpp',
    'navigation_interface.cpp',
]

planner_interface_data_lib = static_library(
    'planner_interface_data_lib',
    planner_interface_data_src,
    include_directories : [ vulcan_inc ],
    link_with : [ local_topo_data_lib ],
    dependencies : [ boost_dep ],
)
