cmake_minimum_required(VERSION 3.5)
project(BinarySearchTreeSearch)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp SequenceST.h FileOps.h)
add_executable(BinarySearchTreeSearch ${SOURCE_FILES})