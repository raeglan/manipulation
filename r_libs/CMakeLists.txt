cmake_minimum_required(VERSION 2.8.3)
project(r_libs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  tf_conversions
)

add_definitions(-std=c++11 -g)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES r_libs_visualization r_libs_conversions
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(r_libs_visualization
   src/VisualizationManager.cpp
)

add_library(r_libs_conversions
   src/R_Conversions.cpp
)

target_link_libraries(r_libs_visualization
  ${catkin_LIBRARIES}
)

target_link_libraries(r_libs_conversions
  ${catkin_LIBRARIES}
)
