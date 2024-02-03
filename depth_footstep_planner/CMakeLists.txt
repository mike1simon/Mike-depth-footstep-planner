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


catkin_package(
  INCLUDE_DIRS include include/sbpl_edit
  LIBRARIES ${PROJECT_NAME} sbpl_edit
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
  include/sbpl_edit
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

set(SBPL_EDIT_FILES
  src/sbpl_edit/Depth2Dgridsearch.cpp
)
add_library(sbpl_edit ${SBPL_EDIT_FILES})
add_dependencies(sbpl_edit ${catkin_EXPORTED_TARGETS})
target_link_libraries(sbpl_edit ${catkin_LIBRARIES} ${SBPL_LIBRARIES})

add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} sbpl_edit ${catkin_LIBRARIES} ${SBPL_LIBRARIES})


add_executable(footstep_planner_node src/footstep_planner.cpp)
add_dependencies(footstep_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_planner_node sbpl_edit ${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES})


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

install(DIRECTORY include/${PROJECT_NAME}/ include/sbpl_edit/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(DIRECTORY config launch maps
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
