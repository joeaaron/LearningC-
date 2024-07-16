cmake_minimum_required(VERSION 3.9)
project(ConeFitting)

find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(ConeFitting ConeFit.cpp)
target_link_libraries(ConeFitting ${PCL_LIBRARIES})
