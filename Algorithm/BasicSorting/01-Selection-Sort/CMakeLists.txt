cmake_minimum_required(VERSION 3.5)
project(SelectionSort)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp Student.h SortTestHelper.h)
add_executable(SelectionSort ${SOURCE_FILES})