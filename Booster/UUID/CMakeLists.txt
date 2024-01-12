cmake_minimum_required(VERSION 3.9)
project(BoostUuid)
set(CMAKE_CXX_STANDARD 11)
set(BOOST_ROOT D:/software/boost_1_73_0)

set(SOURCE_FILES main.cpp ZvUUID.h)
find_package(Boost 1.73.0)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  add_executable(BoostUuid ${SOURCE_FILES})
  target_link_libraries(BoostUuid ${Boost_LIBRARIES})
endif()

