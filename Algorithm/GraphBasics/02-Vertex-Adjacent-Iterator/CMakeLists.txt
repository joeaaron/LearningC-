cmake_minimum_required(VERSION 3.5)
project(VertexAdjacentIterator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp DenseGraph.h SparseGraph.h)
add_executable(VertexAdjacentIterator ${SOURCE_FILES})