set_property(GLOBAL PROPERTY USE_FOLDERS ON)
cmake_minimum_required (VERSION 2.8) 
project (LearningC++)

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )

link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

link_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib")
include_directories("${CMAKE_SOURCE_DIR}/include/")

add_subdirectory(LeetCode)
add_subdirectory(Classic)
add_subdirectory(Algorithm)
add_subdirectory(Booster)
add_subdirectory(Basic)
add_subdirectory(Practice)