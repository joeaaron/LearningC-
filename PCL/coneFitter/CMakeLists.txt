cmake_minimum_required(VERSION 3.9)
project(ConeFitter)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
set(SOURCE_FILES main.cpp ConeFitter.h ConeFitter.cpp FittingBase.h FittingBase.cpp GeometryTypes.h Simplex.h Simplex.cpp GradientConeFitter.h GradientConeFitter.cpp FittingCircle2D.h FittingCircle2D.cpp GradientBase.h GradientBase.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
