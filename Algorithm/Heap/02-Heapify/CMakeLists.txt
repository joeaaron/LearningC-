cmake_minimum_required(VERSION 3.5)
project(Heapify)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp heap.h)
add_executable(Heapify ${SOURCE_FILES})