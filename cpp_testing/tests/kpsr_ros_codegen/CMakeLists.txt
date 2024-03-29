# ****************************************************************************
#
# Klepsydra Core Modules
# Copyright (C) 2019-2031  Klepsydra Technologies AG
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE.md', which is part of this source code package.
#
# NOTICE:  All information contained herein is, and remains the property of Klepsydra
# Technologies AG and its suppliers, if any. The intellectual and technical concepts
# contained herein are proprietary to Klepsydra Technologies AG and its suppliers and
# may be covered by Swiss and Foreign Patents, patents in process, and are protected by
# trade secret or copyright law. Dissemination of this information or reproduction of
# this material is strictly forbidden unless prior written permission is obtained from
# Klepsydra Technologies AG.
#
# ****************************************************************************

cmake_minimum_required(VERSION 2.8.3)
project(kpsr_ros_codegen)

add_definitions(-std=c++11)

find_package(Klepsydra REQUIRED)
# Klepsydra provides the KpsrCodeGen.cmake
include(${KLEPSYDRA_CODE_GENERATOR})

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  kpsr_ros_serialization
  kpsr_ros_core
  message_generation
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(KPSR_LIBRARY_PATH ${CMAKE_INSTALL_LIBDIR})

set(KPSR_INCLUDE_PATH ${CMAKE_INSTALL_INCLUDEDIR})

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a run_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test1" "klepsydra/codegen" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test2" "klepsydra/codegen" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test3" "klepsydra/codegen" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test4" "klepsydra/codegen" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test5" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test5" "klepsydra/codegen" "False" "True")

file(GLOB TEST2_ROS_MESSAGES
  "gen_test2/rosstg/msg/*.msg"
)

file(GLOB TEST3_ROS_MESSAGES
  "gen_test3/rosstg/msg/*.msg"
)

file(GLOB TEST4_ROS_MESSAGES
  "gen_test4/rosstg/msg/*.msg"
)

file(GLOB TEST5_ROS_MESSAGES
  "gen_test5/rosstg/msg/*.msg"
)
file(COPY ${TEST2_ROS_MESSAGES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(COPY ${TEST3_ROS_MESSAGES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(COPY ${TEST4_ROS_MESSAGES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(COPY ${TEST5_ROS_MESSAGES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# # Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
)

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if you package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kpsr_ros_codegen
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
  DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/${PROJECT_NAME}
  gen_test1/poco/include
  gen_test2/poco/include
  gen_test3/poco/include
  gen_test4/poco/include
  gen_test1/rosstg/include
  gen_test2/rosstg/include
  gen_test3/rosstg/include
  gen_test4/rosstg/include
  ../../modules/codegen/include
  ${catkin_INCLUDE_DIRS}
  ${KLEPSYDRA_INCLUDE_DIRS}
)

# link_directories()

# # Declare a C++ library
add_library(kpsr_ros_codegen INTERFACE
)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
add_dependencies(kpsr_ros_codegen kpsr_ros_codegen_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# add_executable(kpsr_ros_codegen_node src/kpsr_ros_codegen_node.cpp)

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(kpsr_ros_codegen_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(kpsr_ros_codegen INTERFACE
  include)

# # Specify libraries to link a library or executable target against
target_link_libraries(kpsr_ros_codegen INTERFACE
  ${catkin_LIBRARIES}
  ${KLEPSYDRA_CORE_LIBRARIES}
)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# install(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables and/or libraries for installation
install(TARGETS kpsr_ros_codegen
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT libs
)

# # Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# # Mark other files for installation (e.g. launch and bag files, etc.)

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
  test/kpsr_ros_codegen_test1.cpp
  test/kpsr_ros_codegen_test2.cpp
  test/kpsr_ros_codegen_test3.cpp
  test/kpsr_ros_codegen_test4.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    atomic)
endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
