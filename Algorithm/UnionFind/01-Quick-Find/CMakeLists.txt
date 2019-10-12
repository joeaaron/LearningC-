cmake_minimum_required(VERSION 3.5)
project(QuickFind)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp UnionFind1.h UnionFindTestHelper.h)
add_executable(QuickFind ${SOURCE_FILES})