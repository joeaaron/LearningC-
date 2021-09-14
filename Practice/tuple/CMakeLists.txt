cmake_minimum_required(VERSION 3.9)
project(PracticeTuple)
set(CMAKE_CXX_STANDARD 11)
set(BOOST_ROOT D:/software/boost_1_73_0)

find_package(Boost 1.73.0)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  aux_source_directory(. sourceList)
  add_executable(PracticeTuple ${sourceList})
  target_link_libraries(PracticeTuple ${Boost_LIBRARIES})
endif()

