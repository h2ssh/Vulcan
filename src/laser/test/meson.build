correlative_scan_matcher_test = executable(
    'correlative_scan_matcher-test',
    'correlative_scan_matcher_test.cpp',
    include_directories : vulcan_inc,
    link_with : [laser_lib, sensor_lib],
    dependencies : [gtest_nomain_dep, lcm_dep],
)

laser_io_test = executable(
    'laser_io-test',
    'laser_io_test.cpp',
    include_directories : vulcan_inc,
    link_with : [laser_lib],
)

test('correlative-scan-matcher-test', correlative_scan_matcher_test)
test('laser-io-test', laser_io_test)
