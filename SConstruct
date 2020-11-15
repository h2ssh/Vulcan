# Base file for the Vulcan build system

# Create the basic environment to be used for all builds. Files that benefit from
# things like loop unrolling can modify CCFLAGS accordingly.

BUILD_LIB_DIR     = '#build/lib'
BUILD_INCLUDE_DIR = '#build/include'
BUILD_BIN_DIR     = '#build/bin'

import os

bare_env = Environment(ENV        = os.environ,             # use the system $PATH variable
                       CCFLAGS    = ['-march=native',
                                     '-pipe',
                                     '-Wall',
                                     '-Werror',
                                     '-Wno-unused-local-typedefs',
                                     '-g'],
                       CXXFLAGS   = ['-std=c++14'],
                       CPPPATH    = [BUILD_INCLUDE_DIR, '#src/', '#external', '#external/cereal/include',
                                     '#external/gnuplot-iostream'],
                       LIBPATH    = [BUILD_LIB_DIR],
                       CPPDEFINES = ['_USE_LCM_',
                                     '-DARMA_DONT_USE_WRAPPER'],
                       tools      = ['default', 'gch'],
                       toolpath   = ['site_scons'])

bare_env["CC"] = os.getenv("CC") or bare_env["CC"]
bare_env["CXX"] = os.getenv("CXX") or bare_env["CXX"]
bare_env["ENV"].update(x for x in os.environ.items() if x[0].startswith("CCC_"))

bare_env.Alias('install', [BUILD_LIB_DIR, BUILD_INCLUDE_DIR, BUILD_BIN_DIR])

# Create the command-line options along with help text
vars = Variables()
vars.Add(BoolVariable('debug', 'Compile in debug mode with -g and -Og', 0))
vars.Add(EnumVariable('robot', 'The type of robot on which the code is run. Options: [\'vulcan\',\'vgo\',\'tufts\',\'lancelot\'] Default:\'vulcan\'', 'vulcan',
                      allowed_values=('vulcan', 'vgo', 'tufts', 'lancelot')))
vars.Add(BoolVariable('camera', 'Compile in support for using a camera (the image_producer module)', 0))
vars.Add(BoolVariable('lcm', 'Run lcm-gen to create new LCM messages. Only needs to be run if new messages are available.', 0))
vars.Add(BoolVariable('log', 'Define LOG_DATA in the preprocessor so internal state of modules will be written to log files', 0))
vars.Add(BoolVariable('test', 'Compile the unit tests', 0))
Help(vars.GenerateHelpText(bare_env))

# Making the release the default because the robot really can't run in realtime without the release optimizations
debug = ARGUMENTS.get('debug', 0)
camera = ARGUMENTS.get('camera', 0)
lcm   = ARGUMENTS.get('lcm', 0)
log   = ARGUMENTS.get('log', 0)
robot = ARGUMENTS.get('robot', 'vulcan')
test  = ARGUMENTS.get('test', 0)

variant_dir = 'build/release'

if int(debug):
    bare_env.Append(CCFLAGS = ['-g', '-Og'])
    variant_dir = 'build/debug'
else:
    bare_env.Append(CCFLAGS = ['-O3'])

if int(log):
    bare_env.Append(CPPDEFINES = ['LOG_DATA'],)

lib_env = bare_env.Clone()
#lib_env.Append(LIBS = ['vulcan_csp', 'vulcan_utils', 'vulcan_robot', 'vulcan_math', 'armadillo', 'rt', 'pthread', 'levmar', 'lapack', 'blas'])

process_env = bare_env.Clone()
process_env.Append(LIBS = ['vulcan_sensors',
                           'vulcan_robot',
                           'vulcan_utils',
                           'vulcan_robot',
                           'vulcan_utils',
                           'vulcan_math',
                           'vulcan_core',
                           'svm',
                           'linear',
                           'armadillo',
                           'rt',
                           'pthread',
                           'levmar',
                           'lapack',
                           'blas',
                           'boost_iostreams',
                           'boost_filesystem',
                           'boost_system'])

mod_env = process_env.Clone()
mod_env.Prepend(LIBS = ['vulcan_lcm', 'vulcan_laser', 'lcm', 'vulcan_system', 'vulcan_vision'])

default_env = mod_env.Clone()

test_env = process_env.Clone()
test_env.Append(CPPPATH='#external/gtest/include')

Export(['lib_env', 'mod_env', 'test_env', 'default_env', 'BUILD_LIB_DIR', 'BUILD_INCLUDE_DIR', 'BUILD_BIN_DIR', 'robot',
        'test', 'camera', 'log', 'lcm'])

if int(test):
    SConscript(['external/gtest/SConscript'])
    test_env.Append(LIBS = ['gtest_main', 'pthread'])

    test_env_no_main = test_env.Clone()
    test_env_no_main.Append(LIBS = ['gtest', 'pthread'])

    Export(['test_env_no_main'])

if int(lcm):
    SConscript('src/lcmtypes/SConscript')
else:
    VariantDir(variant_dir, 'src', duplicate=0)
    SConscript([variant_dir + '/SConscript',
                'external/SConscript',
                'docs/SConscript'])
