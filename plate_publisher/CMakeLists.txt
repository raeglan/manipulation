cmake_minimum_required(VERSION 2.8.3)
project(plate_publisher)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  suturo_perception_msgs
)

add_definitions(-std=c++11 -g)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plate_publisher
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(plate_publisher src/plate_publisher.cpp)

 target_link_libraries(plate_publisher
  ${catkin_LIBRARIES}
)

