CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT(DivideObj)
SET(CMAKE_CXX_FLAGS -std=c++14)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#FIND_PACKAGE(PCL REQUIRED )
#FIND_PACKAGE(Boost REQUIRED)

#FIND_PACKAGE(OpenCV REQUIRED)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )
add_library(DivideObj 
lccp_segmentation.cpp
lccp_segmentation.h
lccp_segmentation.hpp
MarginVoxelLccp.cpp
MarginVoxelLccp.h
supervoxel_clustering.cpp
supervoxel_clustering.h
supervoxel_clustering.hpp
)

target_link_libraries(DivideObj
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${BOOST_LIBRARIES}
    )

