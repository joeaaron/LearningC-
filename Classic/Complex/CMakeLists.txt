cmake_minimum_required(VERSION 3.9)
project(Complex)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES Complex.h Complex_Test.cpp)
add_executable(Complex ${SOURCE_FILES})
