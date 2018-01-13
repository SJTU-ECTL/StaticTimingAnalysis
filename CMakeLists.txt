cmake_minimum_required(VERSION 3.8)
project(StaticTimingAnalysis)
set(CMAKE_CXX_STANDARD 17)

set(lib_directory ${PROJECT_SOURCE_DIR}/lib)
set(bnet_include_directory ${PROJECT_SOURCE_DIR}/include/bnet)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

set(playground_source_files playground.cpp sta.cpp sta.h)

include_directories(${bnet_include_directory})
link_directories(${lib_directory})

add_executable(playground ${playground_source_files})

target_link_libraries(playground bnet)