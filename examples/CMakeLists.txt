cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME examples)
project(${PROJECTNAME})

# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(OpenSoT)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(Boost REQUIRED COMPONENTS system filesystem)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(fcl REQUIRED)
FIND_PACKAGE(geometric_shapes REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${iDynTree_INCLUDE_DIRS} 
                            ${PCL_INCLUDE_DIRS}
                            ${YARP_INCLUDE_DIRS} )

ADD_DEFINITIONS(${PCL_DEFINITIONS})
add_definitions(-DIDYNUTILS_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/../tests/robots/")

add_executable(distance_query distance_query.cpp)
target_link_libraries(distance_query ${Boost_LIBRARIES} ${fcl_LIBRARIES}
                                     ${iDynTree_LIBRARIES} idynutils
                                     ${geometric_shapes_LIBRARIES} ${kdl_parser_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES} ${ros_console_LIBRARIES}
                                     ${tf_LIBRARIES} ${urdf_LIBRARIES})
add_dependencies(distance_query idynutils)

# add required linker flags
