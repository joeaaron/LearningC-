cmake_minimum_required(VERSION 3.9)
project(BoostAny)
set(CMAKE_CXX_STANDARD 11)
set(BOOST_ROOT D:/software/boost_1_73_0/boost_1_73_0)

set(SOURCE_FILES main.cpp)
find_package(Boost 1.73.0)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  add_executable(BoostAny ${SOURCE_FILES})
  target_link_libraries(BoostAny ${Boost_LIBRARIES})
endif()

