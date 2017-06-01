cmake_minimum_required(VERSION 2.8.3)
project(suturo_action_tester)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  r_libs
  roscpp
  rospy
  std_msgs
  suturo_manipulation_msgs
  tf_conversions
  rviz
  visualization_msgs
  giskard
)

add_definitions(-std=c++11 -g)

set(CMAKE_AUTOMOC ON)
## System dependencies are found with CMake's conventions
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Core REQUIRED)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SuturoCreateObject.msg
  AttachObject.msg
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
  std_msgs
  visualization_msgs #   suturo_manipulation_msgs
)


catkin_package(
   DEPENDS
   CATKIN_DEPENDS rviz message_runtime roscpp std_msgs suturo_manipulation_msgs
   INCLUDE_DIRS include
   LIBRARIES
)

###########
## Build ##
###########

qt4_wrap_cpp(MOCS include/suturo_action_tester/SuturoSimPanel.h)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
set(SRC_FILES
   src/SuturoSimPanel.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES} ${MOCS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(suturo_action_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})#suturo_action_tester_generate_messages_cpp)

## Declare a C++ executable
add_executable(controller_checker src/check_controllers.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(suturo_action_tester_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(controller_checker
  ${catkin_LIBRARIES}
)

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
install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
