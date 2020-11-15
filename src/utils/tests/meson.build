##########   Command line test   ##########
cmd_line_test = executable(
    'command-line-test',
    'command_line_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib]
)

test('command-line-test', cmd_line_test)

##########   ICP test   ##########
icp_test = executable(
    'icp-test',
    'icp_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib]
)

test('icp-test', icp_test)

##########   Visibility graph test   ##########
visibility_graph_test = executable(
    'visibility-graph-test',
    'visibility_graph_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('visibility-graph-test', visibility_graph_test)

##########   Compression test   ##########
compression_test = executable(
    'compression-test',
    'compression_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('compression-test', compression_test)

##########   Edit distance test   ##########
edit_distance_test = executable(
    'edit-distance-test',
    'edit_distance_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('edit-distance-test', edit_distance_test)

##########   Pose trace test   ##########
pose_trace_test = executable(
    'pose-trace-test',
    'pose_trace_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('pose-trace-test', pose_trace_test)

##########   Cyclic iterator test   ##########
cyclic_iterator_test = executable(
    'cyclic-iterator-test',
    'cyclic_iterator_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('cyclic-iterator-test', cyclic_iterator_test)

##########   Hough transform test   ##########
hough_transform_test = executable(
    'hough-transform-test',
    'hough_transform_test.cpp',
    include_directories : vulcan_inc,
    link_with : [utils_lib],
    dependencies : [gtest_dep],
)

test('hough-transform-test', hough_transform_test)

##########   Sensor log test   ##########
#sensor_log_test = executable(
#    'sensor-log-test',
#    'command_line_test.cpp',
#    include_directories : vulcan_inc,
#    link_with : [utils_lib, lcm_lib, laser_lib, system_lib],
#    dependencies : [lcm_dep]
#)
#
#test('sensor-log-test', sensor_log_test)
