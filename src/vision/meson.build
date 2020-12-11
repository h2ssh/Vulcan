vision_src = [
    'image_utils.cpp',
    'felzenszwalb_segmenter.cpp',
    'graph_based_segmenter.cpp',
    'wassenberg_segmenter.cpp',
    'image_segmenter.cpp',
    'vision_params.cpp',
]

vision_lib = static_library(
    'vision_lib',
    vision_src,
    include_directories : vulcan_inc,
)
