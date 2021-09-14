cmake_minimum_required(VERSION 3.9)
project(Kd_tree)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable (Kd_tree Kd_tree.cpp)
target_link_libraries (Kd_tree ${PCL_LIBRARIES})
