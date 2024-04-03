cmake_minimum_required(VERSION 3.9)
project(CloudChoose)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable (CloudChoose CloudChoose.cpp)
target_link_libraries (CloudChoose ${PCL_LIBRARIES})
