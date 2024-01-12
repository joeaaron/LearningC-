cmake_minimum_required(VERSION 3.5)
project(BasicMacro)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp macroDef.h)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})