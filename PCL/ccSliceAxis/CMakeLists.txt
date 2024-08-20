cmake_minimum_required(VERSION 3.8)

project(CCSliceAxis)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Zi -Od")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -DEBUG")

#PCL
find_package(PCL 1.14 REQUIRED)
find_package(nanoflann REQUIRED)
find_package(CCCoreLib REQUIRED)

include_directories( ${PCL_INCLUDE_DIRS} )

file( GLOB header_list *h)
file( GLOB source_list *.hpp *.cpp)

#目标文件
add_executable(${PROJECT_NAME} ${header_list})
target_sources(${PROJECT_NAME} PRIVATE ${source_list})

if ( PCL_VERSION VERSION_LESS  1.7 )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS PCL_VER_1_6_OR_OLDER )
endif()

if( PCL_VERSION VERSION_GREATER  1.7 )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS LP_PCL_PATCH_ENABLED )
endif()

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} CCCoreLib::CCCoreLib)

