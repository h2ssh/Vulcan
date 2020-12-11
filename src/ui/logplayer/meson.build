logplayer_ui_src = [
    'logplayer_app.cpp',
    'logplayer_frame.cpp',
    'logplayer_ui.cpp',
]

logplayer_ui = executable(
    'logplayer_ui',
    logplayer_ui_src,
    include_directories : [ vulcan_inc ],
    link_with : [ logplayer_lib, system_lib, ui_common_lib, ui_components_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep, ui_dep ],
    install : true,
)
