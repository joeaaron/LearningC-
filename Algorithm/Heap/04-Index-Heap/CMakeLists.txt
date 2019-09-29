cmake_minimum_required(VERSION 3.5)
project(IndexHeap)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp SortTestHelper.h)
add_executable(IndexHeap ${SOURCE_FILES})