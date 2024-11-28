cmake_minimum_required(VERSION 3.9)
project(CloudConnect)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable (${PROJECT_NAME} CloudConnect.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
