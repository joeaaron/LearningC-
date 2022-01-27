cmake_minimum_required(VERSION 3.9)
project(Utility)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES Utility.h Utility.cpp main.cpp)
add_executable(Utility ${SOURCE_FILES})
