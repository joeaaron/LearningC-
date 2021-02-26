cmake_minimum_required(VERSION 3.5)
project(ReadGraph)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp ReadGraph.h DenseGraph.h SparseGraph.h)
add_executable(ReadGraph ${SOURCE_FILES})