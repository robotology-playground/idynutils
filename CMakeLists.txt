# 
#  Copyright (C) 2014 Walkman
#  Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi
#  email: mirko.ferrati@gmail.com, enrico.mingo@iit.it, alessio.rocchi@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(idynutils)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(kdl_conversions REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(fcl REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED COMPONENTS    common
                                            filters
                                            surface
                                            search
                                            io)
                                            
# on xenial there is a bug ATM wo that fcl is found but fcl_LIBRARIES is not
if(NOT DEFINED fcl_LIBRARIES)
  set(fcl_LIBRARIES "fcl")
endif(NOT DEFINED flc_LIBRARIES)

#if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wmissing-field-initializers -W -Wunused -Wuninitialized -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing  -Werror=address -Werror=parentheses " )
#endif(CMAKE_BUILD_TYPE STREQUAL "Debug")


# Detect Linux Distribution 
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     if(EXISTS "/etc/lsb-release")
         execute_process(COMMAND cat /etc/lsb-release
                         COMMAND grep DISTRIB_CODENAME
                         COMMAND awk -F= "{ print $2 }"
                         COMMAND tr "\n" " "
                         COMMAND sed "s/ //"
                         OUTPUT_VARIABLE UBUNTU_VERSION
                         RESULT_VARIABLE UBUNTU_VERSION_RESULT
         )
     endif(EXISTS "/etc/lsb-release")
     
     message(STATUS "Configuring idynutils to be built on Ubuntu ${UBUNTU_VERSION}")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${UBUNTU_VERSION} MATCHES "xenial")
     message(STATUS "Adding CXX support")
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
endif(${UBUNTU_VERSION} MATCHES "xenial")

INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS}
                            ${PCL_INCLUDE_DIRS} ${moveit_core_INCLUDE_DIRS} )

# for every file in idynutils_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE idynutils_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include/idynutils" *.h*)
file(GLOB_RECURSE idynutils_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

ADD_LIBRARY(idynutils SHARED    src/cartesian_utils.cpp
                                src/collision_utils.cpp
                                src/ComanUtils.cpp
                                src/convex_hull.cpp
                                src/idynutils.cpp
                                src/octomap_utils.cpp
                                src/RobotUtils.cpp
                                src/tests_utils.cpp
                                src/WalkmanUtils.cpp
                                src/yarp_ft_interface.cpp
                                src/yarp_IMU_interface.cpp
                                src/yarp_single_chain_interface.cpp
                                ${idynutils_INCLUDES})

TARGET_LINK_LIBRARIES(idynutils PRIVATE ${orocos_kdl_LIBRARIES}
                                        ${urdf_LIBRARIES}
                                        ${srdfdom_LIBRARIES} 
                                        ${kdl_parser_LIBRARIES}
                                        ${iDynTree_LIBRARIES}
                                        ${kdl_conversions_LIBRARIES}
                                        ${YARP_LIBRARIES}
                                        ${fcl_LIBRARIES}
                                        ${moveit_core_LIBRARIES}
                                        ${PCL_LIBRARIES})

##########################################################################
# use YCM to export idynutils so that it can be found using find_package #
# ########################################################################

set(VARS_PREFIX "IDYNUTILS")

set(IDYNUTILS_MAJOR_VERSION 1)
set(IDYNUTILS_MINOR_VERSION 0)
set(IDYNUTILS_PATCH_VERSION 0)
set(IDYNUTILS_VERSION ${IDYNUTILS_MAJOR_VERSION}.${IDYNUTILS_MINOR_VERSION}.${IDYNUTILS_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(idynutils PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                            "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(idynutils PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                SOVERSION ${${VARS_PREFIX}_VERSION})

target_link_libraries(idynutils LINK_PUBLIC ${moveit_core_LIBRARIES})

message("fcl version: ${fcl_VERSION} ${fcl_MAJOR_VERSION}")


install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS idynutils  
        EXPORT idynutils
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)
        
#enabling it will add all idynutils dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idynutils)


include(InstallBasicPackageFiles)
install_basic_package_files(idynutils   VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

add_subdirectory(doc)

set(COMPILE_EXAMPLES FALSE CACHE BOOL "Compile the examples?")
if(COMPILE_EXAMPLES)
	add_subdirectory(examples)
endif(COMPILE_EXAMPLES)

set(COMPILE_BINDINGS TRUE CACHE BOOL "Compile python bindings?")
if(COMPILE_BINDINGS)
    add_subdirectory(bindings)
endif(COMPILE_BINDINGS)

#######################
# Add Testing target  #
#######################
enable_testing()
add_subdirectory(tests)
