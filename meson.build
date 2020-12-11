project('vulcan', 'cpp', 'c',
    default_options : ['cpp_std=c++17'])

add_project_arguments('-Wno-non-virtual-dtor', #needed to compile cereal
                      '-Wno-unused-parameter',  # needed for LCM handlers and elsewhere
                      '-Wno-deprecated-copy', # needed for wxWidgets
                      '-Wno-ignored-qualifiers', # needed for wxWidgets
                      '-Wno-cast-function-type', # needed for wxWidgets
                      '-DARMA_DONT_USE_WRAPPER', # needed to link armadillo w/o cmake wrapper
                      language : 'cpp',)

add_project_arguments('-Wno-pedantic',  # needed for LCM auto-gen C files
                      '-Wno-unused-parameter',  # needed for LCM handlers and elsewhere
                      language : 'c',)

lcm_dep = dependency('lcm')
thread_dep = dependency('threads')
boost_dep = dependency('boost', modules : ['iostreams', 'filesystem', 'system'])
lapack_dep = dependency('lapack')
blas_dep = dependency('blas')
gtest_dep = dependency('gtest', main : true, required : false)
gtest_nomain_dep = dependency('gtest', main : false, required : false)

levmar_dep = dependency('levmar')
nlopt_dep = dependency('nlopt')

arma_base_dep = dependency('armadillo')
# manually define the armadillo dependency to also link in blas and lapack since we aren't using cmake wrapper
arma_dep = declare_dependency(dependencies : [ lapack_dep, blas_dep, arma_base_dep] )

vulcan_inc = include_directories('src',
                                 'external',
                                 'external/cereal/include',
                                 'external/gnuplot-iostream',)

subdir('src')
