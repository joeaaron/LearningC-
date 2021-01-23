cmake_minimum_required(VERSION 3.9)
project(BasicThis)
set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. sourceList)
add_executable(BasicThis ${sourceList})
