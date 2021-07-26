cmake_minimum_required(VERSION 3.9)
project(Utility)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp Utility.cpp Utility.h)
add_executable(Utility ${SOURCE_FILES})
