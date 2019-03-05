
cmake_minimum_required(VERSION 2.8)
project(pgm)

set(CMAKE_CXX_FLAGS "-DIMGUI_IMPL_OPENGL_LOADER_GLAD -O2 -g -std=c++14 -Wall -Wextra")
set(CMAKE_CXX_COMPILER g++)

include_directories("../src/")

find_package(glfw3 REQUIRED)

file(
    GLOB_RECURSE
    source_files
    ../src/*.[ch]pp ../external/*.[ch]pp ../external/*.h ../shaders/*.glsl
)
add_executable(pgm ${source_files})

target_link_libraries(pgm glfw dl)
