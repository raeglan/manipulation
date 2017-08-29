cmake_minimum_required(VERSION 2.8.3)
project(suturo_action_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  giskard_core
  giskard_suturo_parser
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  suturo_manipulation_msgs
  tf_conversions
  eigen_conversions
  r_libs
  control_msgs
  pluginlib
  octomap_msgs
)

add_definitions(-std=c++11 -g)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS suturo_manipulation_msgs giskard_core r_libs roscpp tf_conversions
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${r_libs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


## Declare a C++ executable
 add_executable(suturo_action_server src/main_server.cpp src/GiskardActionServer.cpp src/CollisionScene.cpp )
 add_dependencies(suturo_action_server r_libs_visualization r_libs_conversions suturo_manipulation_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(suturo_action_server
  ${catkin_LIBRARIES}
  r_libs_visualization
  r_libs_conversions
  yaml-cpp
  ${OCTOMAP_LIBRARIES}
)

add_executable(client_test src/client_test.cpp)

add_dependencies(client_test 
  graspkard_msgs
  suturo_manipulation_msgs_generate_messages_cpp
  #${graspkard_EXPORTED_TARGETS}
)

target_link_libraries(client_test 
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_suturo_action_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
