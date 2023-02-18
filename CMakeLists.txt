cmake_minimum_required(VERSION 3.5)

project(SegmentMap LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)



FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(PCL REQUIRED )
FIND_PACKAGE(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/YamlLib/include
    )

add_subdirectory(${PROJECT_SOURCE_DIR}/DivideObj)
add_executable(SegmentMap SegmentMap.cpp
    ReadFile.hpp
    )

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_libraries(SegmentMap
        #    ${BOOST_LIBRARIES}
        #    ${PCL_LIBRARIES}
        #    ${OpenCV_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/YamlLib/Release/libyaml-cppmd.lib
        DivideObj
        Qt5::Core
        )
else(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_libraries(SegmentMap
        #    ${BOOST_LIBRARIES}
        #    ${PCL_LIBRARIES}
        #    ${OpenCV_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/YamlLib/Debug/libyaml-cppmdd.lib
        DivideObj
        Qt5::Core
        )
endif(CMAKE_BUILD_TYPE STREQUAL "Release")

