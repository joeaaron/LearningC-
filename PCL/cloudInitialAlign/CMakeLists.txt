cmake_minimum_required(VERSION 3.9)
project(CloudInitialAlign)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.14 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable (${PROJECT_NAME} CloudInitialAlign.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
