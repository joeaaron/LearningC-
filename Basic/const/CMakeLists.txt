cmake_minimum_required(VERSION 3.9)
project(BasicConst)
set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. sourceList)
add_executable(BasicConst ${sourceList})
