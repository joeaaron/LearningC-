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

option(BUILD_AlGORITHM	"Build Algorithm"  OFF)
option(BUILD_PCL	"Build PCL"  OFF)
option(BUILD_LEETCODE	"Build LeetCode"  OFF)
option(BUILD_CLASSIC	"Build Classic"  OFF)
option(BUILD_PRIMER	"Build Primer"  OFF)
option(BUILD_BOOSTER	"Build Booster"  OFF)
option(BUILD_BASIC	"Build Basic"  OFF)
option(BUILD_PRACTICE	"Build Practice"  OFF)
option(BUILD_STL	"Build STL"  OFF)

if(BUILD_AlGORITHM)
	add_subdirectory(Algorithm)
endif(BUILD_AlGORITHM)

if(BUILD_PCL)
	add_subdirectory(PCL)
endif(BUILD_PCL)

if(BUILD_LEETCODE)
	add_subdirectory(LeetCode)
endif(BUILD_LEETCODE)

if(BUILD_CLASSIC)
	add_subdirectory(Classic)
endif(BUILD_CLASSIC)

if(BUILD_PRIMER)
	add_subdirectory(Primer)
endif(BUILD_PRIMER)

if(BUILD_BOOSTER)
	add_subdirectory(Booster)
endif(BUILD_BOOSTER)

if(BUILD_BASIC)
	add_subdirectory(Basic)
endif(BUILD_BASIC)

if(BUILD_PRACTICE)
	add_subdirectory(Practice)
endif(BUILD_PRACTICE)

if(BUILD_STL)
	add_subdirectory(STL)
endif(BUILD_STL)