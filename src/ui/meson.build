wx_dep = dependency('wxwidgets', version : '>=3.0', modules : [ 'core', 'base', 'gl', 'aui', 'adv' ])
gl_dep = dependency('gl')
glu_dep = dependency('glu')

ui_dep = declare_dependency(dependencies : [ wx_dep, gl_dep, glu_dep ] )

subdir('common')        # ui_common_lib
subdir('components')    # ui_components_lib
subdir('debug')
subdir('navigation')
subdir('mapeditor')
subdir('logplayer')
