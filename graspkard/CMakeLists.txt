cmake_minimum_required(VERSION 2.8.3)
project(graspkard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  giskard
  message_generation
  message_runtime
  r_libs
  roscpp
  tf_conversions
)

add_definitions(-std=c++11 -g)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #  ${r_libs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(graspkard_node src/pr2_r_vel_controller.cpp)

target_link_libraries(graspkard_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
