math_lib_src = [
    'angle_range.cpp',
    'beta_distribution.cpp',
    'boundary.cpp',
    'dbscan.cpp',
    'derivatives.cpp',
    'discrete_gaussian.cpp',
    'em.cpp',
    'exponential_distribution.cpp',
    'gamma_distribution.cpp',
    'geometry/geometric_circle_fitting.cpp',
    'geometry/rectangle_fitting.cpp',
    'geometry/shape_intersection_area.cpp',
    'geometry/circle_fitting.cpp',
    'geometry/algebraic_circle_fitting.cpp',
    'geometry/circle_fitting_utils.cpp',
    'geometry/line_intersections.cpp',
    'graph/connected_components.cpp',
    'graph/all_paths.cpp',
    'interpolation.cpp',
    'kmeans.cpp',
    'mahalanobis.cpp',
    'ransac.cpp',
    'regression.cpp',
    'savitzky_golay.cpp',
    'softmax.cpp',
    'truncated_gaussian_distribution.cpp',
    't_test.cpp',
    'univariate_distribution.cpp',
    'univariate_distribution_estimator_impl.cpp',
    'univariate_gaussian.cpp',
    'z_test.cpp',
]

math_lib = static_library(
    'math_lib',
    math_lib_src,
    include_directories : vulcan_inc,
    link_with : [core_lib]
)

subdir('geometry/tests')
subdir('tests')
