cmake_minimum_required(VERSION 2.8.3)
project(shm_humanoid_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message("cmake module path:" ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED COMPONENTS
  m3ctrl_msgs
  nav_msgs
  rosconsole
  roscpp
  rospy
  tf
)
## YAML CPP 
find_package(PkgConfig REQUIRED)
#find_package(YamlCpp  REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
message("yaml-cpp version:" ${YAMLCPP_VERSION})
if(${YAMLCPP_VERSION} VERSION_LESS "0.5")
set_property(DIRECTORY
                PROPERTY COMPILE_DEFINITIONS "YAMLCPP_03")
endif(${YAMLCPP_VERSION} VERSION_LESS "0.5")


find_package(M3rt REQUIRED)
find_package(M3 REQUIRED COMPONENTS HARDWARE) # for protobuf messages
find_package(RTAI REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   m3ctrl_msgs#   nav_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shm_generic
  CATKIN_DEPENDS m3ctrl_msgs nav_msgs rosconsole roscpp rospy tf
#  DEPENDS system_lib
)


###########
## Build ##
###########


SET(LIBS ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${CMAKE_DL_LIBS} ${RTAI_LIBRARY} ${M3_LIBRARIES} ${M3RT_LIBRARIES})
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${YAMLCPP_INCLUDE_DIRS}
  ${RTAI_INCLUDE_DIR}
${M3RT_INCLUDE_DIRS}
${M3RT_INCLUDE_DIRS}
)





#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(shm_humanoid_controller src/shm_humanoid_controller.cpp)
add_executable(move_humanoid src/move_humanoid.cpp)
add_executable(move_gripper src/move_gripper.cpp)
add_executable(pose_right_arm src/pose_right_arm.cpp)
add_executable(gravity_right_arm src/gravity_right_arm.cpp)
#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(shm_humanoid_controller ${LIBS})
target_link_libraries(move_humanoid ${LIBS})
target_link_libraries(move_gripper ${LIBS})
target_link_libraries(pose_right_arm ${LIBS})
target_link_libraries(gravity_right_arm ${LIBS})
## Declare a cpp library
# add_library(shm_generic
#   src/${PROJECT_NAME}/shm_generic.cpp
# )

## Declare a cpp executable
# add_executable(shm_generic_node src/shm_generic_node.cpp)


## Wait for the M3JointCmd.h to be generated

add_dependencies(shm_humanoid_controller M3JointCmd.h)
add_dependencies(move_humanoid M3JointCmd.h)
add_dependencies(move_gripper M3JointCmd.h)
add_dependencies(pose_right_arm M3JointCmd.h)
add_dependencies(gravity_right_arm M3JointCmd.h)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(shm_generic_node shm_generic_generate_messages_cpp)
add_dependencies(shm_humanoid_controller m3ctrl_msgs_gencpp)
add_dependencies(move_humanoid m3ctrl_msgs_gencpp)
add_dependencies(move_gripper m3ctrl_msgs_gencpp)
add_dependencies(pose_right_arm m3ctrl_msgs_gencpp)
add_dependencies(gravity_right_arm m3ctrl_msgs_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(shm_generic_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS shm_generic shm_generic_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_shm_generic.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
