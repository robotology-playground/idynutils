cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

find_package(Boost REQUIRED COMPONENTS system filesystem)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(fcl REQUIRED)
FIND_PACKAGE(geometric_shapes REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(rosbag REQUIRED)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

# on xenial there is a bug ATM wo that fcl is found but fcl_LIBRARIES is not
if(NOT DEFINED fcl_LIBRARIES)
  set(fcl_LIBRARIES "fcl")
endif(NOT DEFINED flc_LIBRARIES)

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

# compite GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

if(YCM_EP_DEVEL_MODE_idynutils)
  add_custom_command( TARGET idynutils POST_BUILD
                      COMMAND ${CMAKE_CTEST_COMMAND}
                      MAIN_DEPENDENCY idynutils
                      DEPENDS   CartesianUtilsTest
                                CollisionUtilsTest
                                iDynUtilsTest
                                interfacesTest
                                RobotUtilsTest
                                testUtilsTest
                                YSCITest)
endif()
SET(TestLibs  ${GTEST_BOTH_LIBRARIES} ${iDynTree_LIBRARIES} idynutils 
              ${kdl_parser_LIBRARIES}
              ${moveit_core_LIBRARIES}  ${orocos_kdl_LIBRARIES} 
              ${PCL_LIBRARIES} ${urdf_LIBRARIES} 
              ${srdfdom_LIBRARIES} ${YARP_LIBRARIES})

add_definitions(-DIDYNUTILS_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

ADD_EXECUTABLE(interfacesTest test_interfaces.cpp)
TARGET_LINK_LIBRARIES(interfacesTest ${TestLibs})
add_dependencies(interfacesTest GTest-ext idynutils)

ADD_EXECUTABLE(CartesianUtilsTest     cartesian_utils_tests.cpp)
TARGET_LINK_LIBRARIES(CartesianUtilsTest ${TestLibs})
add_dependencies(CartesianUtilsTest GTest-ext idynutils)

ADD_EXECUTABLE(CollisionUtilsTest     collision_utils_tests.cpp)
TARGET_LINK_LIBRARIES(CollisionUtilsTest ${TestLibs} ${fcl_LIBRARIES})
add_dependencies(CollisionUtilsTest GTest-ext idynutils)

ADD_EXECUTABLE(iDynUtilsTest    idyn_utils_tests.cpp)
TARGET_LINK_LIBRARIES(iDynUtilsTest ${TestLibs} ${rosbag_LIBRARIES})
add_dependencies(iDynUtilsTest GTest-ext idynutils)

ADD_EXECUTABLE(RobotUtilsTest    robot_utils_tests.cpp)
TARGET_LINK_LIBRARIES(RobotUtilsTest ${TestLibs})
add_dependencies(RobotUtilsTest GTest-ext idynutils)

ADD_EXECUTABLE(YSCITest    yarp_single_chain_interface_tests.cpp)
TARGET_LINK_LIBRARIES(YSCITest ${TestLibs})
add_dependencies(YSCITest GTest-ext idynutils)

ADD_EXECUTABLE(testUtilsTest     tests_utils_tests.cpp)
TARGET_LINK_LIBRARIES(testUtilsTest ${TestLibs})
add_dependencies(testUtilsTest GTest-ext idynutils)

add_definitions(-DIDYNUTILS_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")
add_definitions(-DIDYNUTILS_TESTS_DATA_DIR="${CMAKE_CURRENT_BINARY_DIR}/data/")

add_test(NAME cartesian_utils_tests COMMAND CartesianUtilsTest)
add_test(NAME collision_utils_tests COMMAND CollisionUtilsTest)
add_test(NAME idyn_utils_tests COMMAND iDynUtilsTest)
add_test(NAME robot_utils_tests COMMAND RobotUtilsTest)
add_test(NAME tests_utils_tests COMMAND testUtilsTest)
add_test(NAME yarp_single_chain_interface_tests COMMAND YSCITest)

add_custom_target(copy_robot_model_files ALL
                  ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/robots" "${CMAKE_CURRENT_BINARY_DIR}/robots")

add_custom_target(copy_robot_data_files ALL
                  ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data" "${CMAKE_CURRENT_BINARY_DIR}/data")
