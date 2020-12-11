mapeditor_src = [
    'classification_test_results_dialog.cpp',
    'gateway_classifier_test_dialog.cpp',
    'gateway_editor.cpp',
    'global_topo_editor_panel.cpp',
    'global_topo_editor_widget.cpp',
    'import_image_dialog.cpp',
    'local_topo_editor_panel.cpp',
    'local_topo_editor_widget.cpp',
    'lpm_editing.cpp',
    'map_editor_app.cpp',
    'map_editor.cpp',
    'map_editor_frame.cpp',
    'metric_editor_panel.cpp',
    'metric_editor_widget.cpp',
]

mapeditor = executable(
    'map_editor',
    mapeditor_src,
    include_directories : [ vulcan_inc ],
    link_with : [ lcm_lib, local_topo_func_lib, local_topo_training_lib, system_lib, ui_common_lib, ui_components_lib ],
    dependencies : [ boost_dep, lcm_dep, thread_dep, ui_dep ],
    install : true,
)
