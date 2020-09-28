cmake_minimum_required(VERSION 3.9)
project(4Pract)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES solution.cpp BigInteger.h strings.h strings.cpp)
add_executable(4Pract ${SOURCE_FILES})
