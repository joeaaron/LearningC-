cmake_minimum_required(VERSION 3.5)
project(BookStore)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp Sales_item.h)
add_executable(BookStore ${SOURCE_FILES})