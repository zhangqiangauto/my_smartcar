cmake_minimum_required(VERSION 2.8.3)
project(static_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES static_map
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(static_map_node
  src/static_map.cpp
)

add_dependencies(static_map_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(static_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(densy_map
  src/static_map_densy.cpp
)

add_dependencies(densy_map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(densy_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(dynamic_pcmap_loader
  src/dynamic_pcmap_loader.cpp
)

add_dependencies(dynamic_pcmap_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dynamic_pcmap_loader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
