cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_movebaseflex)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wno-deprecated-declarations)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  move_base_flex_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
  actionlib
  mongodb_store

)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs move_base_flex_msgs mongodb_store diagnostic_msgs rosplan_planning_system message_runtime
  DEPENDS
)

###########
## Build ##
###########


## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare cpp executables
add_executable(rpmovebaseflex src/RPMoveBaseFlex.cpp)
add_dependencies(rpmovebaseflex ${catkin_EXPORTED_TARGETS})
add_dependencies(rpmovebaseflex move_base_flex_msgs_generate_messages_cpp)
## Specify libraries against which to link a library or executable target
target_link_libraries(rpmovebaseflex ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS rpmovebaseflex
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
