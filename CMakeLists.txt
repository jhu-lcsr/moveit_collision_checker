cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_checker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_ros_planning urdf roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x")

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
add_service_files(
  FILES
  CheckState.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs
)

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
  INCLUDE_DIRS include
#  LIBRARIES moveit_collision_checker
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

## Declare a cpp library
add_library(moveit_collision_checker
  moveit_collision_checker.cpp
)
target_link_libraries(moveit_collision_checker ${catkin_LIBRARIES})

add_executable(mcc_test mcc_test.cpp)
target_link_libraries(mcc_test moveit_collision_checker ${catkin_LIBRARIES})

add_executable(mcc_node mcc_node.cpp)
target_link_libraries(mcc_node moveit_collision_checker ${catkin_LIBRARIES})
add_dependencies(mcc_node ${PROJECT}_generate_messages_cpp)

#python_add_module(moveit_collision_checker_py moveit_collision_checker_py.cpp)



add_library(moveit_collision_checker_py SHARED moveit_collision_checker_py.cpp )
target_link_libraries(moveit_collision_checker_py ${Boost_LIBRARIES} moveit_collision_checker)
set_target_properties(moveit_collision_checker_py PROPERTIES PREFIX "" )
message(STATUS "Output: ${CATKIN_DEVEL_PREFIX}/${PYTHON_SITE_PACKAGES}")
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(plat_specific=True, prefix=''))" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
set_target_properties(moveit_collision_checker_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${PYTHON_SITE_PACKAGES}")

## Declare a cpp executable
# add_executable(moveit_collision_checker_node src/moveit_collision_checker_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(moveit_collision_checker_node moveit_collision_checker_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(moveit_collision_checker_node
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
# install(TARGETS moveit_collision_checker moveit_collision_checker_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_moveit_collision_checker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
