install_subdir(
    'default',
    install_dir : get_option('bindir'),
    strip_directory : true,
)

if get_option('platform') != 'none'
    install_subdir(
        get_option('platform'),
        install_dir : get_option('bindir'),
        strip_directory : true,
    )
endif
