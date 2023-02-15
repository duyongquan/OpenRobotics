find_program(SPHINX_EXECUTABLE
             NAMES sphinx-build
             PATHS
               /usr/bin
             DOC "Sphinx")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sphinx DEFAULT_MSG SPHINX_EXECUTABLE)