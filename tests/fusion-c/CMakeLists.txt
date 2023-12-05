# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10)
# Project name
project(fusion-c)

# add_definitions(-DFUSION_USE_NORMAL_SQRT)

include_directories("../../fusion-c/Fusion")

add_subdirectory("../../fusion-c/Fusion" "Fusion")

# Add the source files
add_executable(fusion-c main.c)

target_link_libraries(fusion-c PRIVATE "Fusion")
