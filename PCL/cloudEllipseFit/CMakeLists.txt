cmake_minimum_required(VERSION 3.9)
project(EllipseFitting)

find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} EllipseFit.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})
