cmake_minimum_required(VERSION 3.0.2)
project(depth_footstep_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  depthmap_humanoid_msgs
  geometry_msgs
  humanoid_nav_msgs
  nav_msgs
  rosconsole
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
set(dynamicEDT3D_DIR "/opt/ros/noetic/share/dynamicEDT3D")
set(DYNAMICEDT3D_LIBRARIES "/opt/ros/noetic/lib/libdynamicedt3d.so")
find_package(dynamicEDT3D REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   depthmap_humanoid_msgs#   geometry_msgs#   humanoid_nav_msgs#   nav_msgs#   sensor_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth_footstep_planner
#  CATKIN_DEPENDS actionlib angles depthmap_humanoid_msgs geometry_msgs humanoid_nav_msgs nav_msgs rosconsole roscpp sensor_msgs tf visualization_msgs
#  DEPENDS system_lib
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    actionlib
    rosconsole
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    humanoid_nav_msgs
    tf
    angles
    depthmap_humanoid_msgs
  DEPENDS OpenCV SBPL
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)
set(FOOTSTEP_PLANNER_FILES
  src/FootstepPlanner.cpp
#  src/FootstepNavigation.cpp
  src/FootstepPlannerNode.cpp
  src/FootstepPlannerEnvironment.cpp
  src/Footstep.cpp
  src/PlanningState.cpp
  src/Heuristic.cpp
  src/helper.cpp
  src/PathCostHeuristic.cpp
  src/PlanningStateChangeQuery.cpp
  src/State.cpp
)

add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES})

add_executable(footstep_planner_node src/footstep_planner.cpp)
add_dependencies(footstep_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES})


################################################################################
# Install
################################################################################
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS footstep_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(DIRECTORY config launch maps
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
