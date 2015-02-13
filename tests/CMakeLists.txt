cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

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
                                iDynUtilsTest
                                interfacesTest
                                RobotUtilsTest
                                testUtilsTest
                                YSCITest)
endif()

SET(TestLibs  ${GTEST_BOTH_LIBRARIES} ${iDynTree_LIBRARIES} idynutils 
              ${kdl_codyco_LIBRARIES} ${kdl_parser_LIBRARIES}
              ${moveit_core_LIBRARIES}  ${orocos_kdl_LIBRARIES} 
              ${PCL_LIBRARIES} ${urdf_LIBRARIES} 
              ${srdfdom_LIBRARIES} ${YARP_LIBRARIES} )

ADD_EXECUTABLE(interfacesTest test_interfaces.cpp)
TARGET_LINK_LIBRARIES(interfacesTest ${TestLibs})
add_dependencies(interfacesTest GTest-ext idynutils)

ADD_EXECUTABLE(CartesianUtilsTest     cartesian_utils_tests.cpp)
TARGET_LINK_LIBRARIES(CartesianUtilsTest ${TestLibs})
add_dependencies(CartesianUtilsTest GTest-ext idynutils)

ADD_EXECUTABLE(iDynUtilsTest    idyn_utils_tests.cpp)
TARGET_LINK_LIBRARIES(iDynUtilsTest ${TestLibs})
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

add_test(NAME cartesian_utils_tests COMMAND CartesianUtilsTest)
add_test(NAME idyn_utils_tests COMMAND iDynUtilsTest)
add_test(NAME robot_utils_tests COMMAND RobotUtilsTest)
add_test(NAME tests_utils_tests COMMAND testUtilsTest)
add_test(NAME yarp_single_chain_interface_tests COMMAND YSCITest)
