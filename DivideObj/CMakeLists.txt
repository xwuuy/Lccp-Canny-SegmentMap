CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT(DivideObj)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_CXX_FLAGS -std=c++14)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

FIND_PACKAGE( PCL REQUIRED )
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}
    /usr/local/include/yaml-cpp
    )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES})

add_library(DivideObj 
lccp_segmentation.cpp
lccp_segmentation.h
lccp_segmentation.hpp
MarginVoxelLccp.cpp
MarginVoxelLccp.h
supervoxel_clustering.cpp
supervoxel_clustering.h
supervoxel_clustering.hpp)
