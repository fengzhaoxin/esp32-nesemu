# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add_compile_options (-fdiagnostics-color=always)
set(CFLAGS "${CFLAGS} -Wno-error=char-subscripts -Wno-error=attributes -DNOFRENDO_DEBUG")
set(CFLAGS "${CFLAGS} -Wno-implicit-fallthrough")

project(nesemu)