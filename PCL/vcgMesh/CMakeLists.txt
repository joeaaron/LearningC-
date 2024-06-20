cmake_minimum_required(VERSION 3.8)

project(VcgMesh)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Zi -Od")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -DEBUG")

#PCL
find_package(PCL 1.14 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
set(SOURCES 
	vcgMesh.cpp
	${VCG_INCLUDE_DIRS}/wrap/ply/plylib.cpp)


add_executable(VcgMesh
	${SOURCES})

target_link_libraries (VcgMesh ${PCL_LIBRARIES})