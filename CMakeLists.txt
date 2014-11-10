# Copyright: (C) 2014 Walkman Consortium
# Authors: Mirko Ferrati, Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(idynutils)

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

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(kdl_codyco REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED COMPONENTS    #common
                                            #features
                                            filters
                                            #kdtree
                                            #io
                                            #sample_consensus
                                            #search
                                            #segmentation
                                            surface)

INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS}
                            ${kdl_codyco_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE idynutils_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include/idynutils" *.h*)
file(GLOB_RECURSE idynutils_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

ADD_LIBRARY(idynutils SHARED    src/cartesian_utils.cpp
                                src/comanutils.cpp
                                src/convex_hull.cpp
                                src/idynutils.cpp
                                src/tests_utils.cpp
                                src/yarp_ft_interface.cpp
                                src/yarp_IMU_interface.cpp
                                src/yarp_single_chain_interface.cpp
                                ${idynutils_INCLUDES})

TARGET_LINK_LIBRARIES(idynutils PRIVATE ${iDynTree_LIBRARIES} ${kdl_codyco_LIBRARIES}
                                        ${kdl_parser_LIBRARIES} ${moveit_core_LIBRARIES} 
                                        ${orocos_kdl_LIBRARIES} ${srdfdom_LIBRARIES} 
                                        ${urdf_LIBRARIES} ${YARP_LIBRARIES}
                                        ${PCL_LIBRARIES})

########################################################################
# use YCM to export OpenSoT so taht it can be found using find_package #
########################################################################

set(VARS_PREFIX "IDYNUTILS")

set(IDYNUTILS_MAJOR_VERSION 0)
set(IDYNUTILS_MINOR_VERSION 0)
set(IDYNUTILS_PATCH_VERSION 1)
set(IDYNUTILS_VERSION ${IDYNUTILS_MAJOR_VERSION}.${IDYNUTILS_MINOR_VERSION}.${IDYNUTILS_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(idynutils PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                            "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(idynutils PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS idynutils  
        EXPORT idynutils
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)
        
#enabling it will add all OpenSoT dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idynutils)


include(InstallBasicPackageFiles)
install_basic_package_files(idynutils   VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

add_subdirectory(doc)
add_subdirectory(examples)

#######################
# Add Testing target  #
#######################
enable_testing()
add_subdirectory(tests)
