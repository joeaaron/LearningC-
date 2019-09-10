cmake_minimum_required(VERSION 3.5)
project(InsertionSort)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp SelectionSort.h SortTestHelper)
add_executable(InsertionSort ${SOURCE_FILES})