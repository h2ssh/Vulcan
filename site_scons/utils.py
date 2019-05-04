import os.path

# Snagged from: http://stackoverflow.com/questions/12518715/how-do-i-filter-an-scons-glob-result
def FilteredGlob(env, pattern, omit=[], ondisk=True, source=False, strings=False):
    return filter(
      lambda f: os.path.basename(f.path) not in omit,
      env.Glob(pattern))

def NoParamsGlob(env, pattern):
    return FilteredGlob(env, pattern, 'params.cpp')
