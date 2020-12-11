logplayer_lib_src = [
    'beeson_reader.cpp',
    'carmen_reader.cpp',
    'log_loader.cpp',
    'log_player.cpp',
    'log_reader.cpp',
    'params.cpp',
    'player_reader.cpp',
    'saphira_reader.cpp',
]

logplayer_module_src = [
    'main.cpp',
]


logplayer_lib = static_library(
    'logplayer_lib',
    logplayer_lib_src,
    include_directories : vulcan_inc,
    link_with : [ lcm_lib, system_lib, utils_lib, ],
)

logplayer = executable(
    'logplayer',
    logplayer_module_src,
    include_directories : vulcan_inc,
    link_with : [ logplayer_lib ],
    install : true,
)
