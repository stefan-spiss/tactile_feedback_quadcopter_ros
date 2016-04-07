cmake_minimum_required(VERSION 2.8.3)
project(tactile_feedback_quadcopter_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_srvs
  ardrone_autonomy
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_srvs ardrone_autonomy
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


set(SOURCES
  src/ObstacleObserver.cpp 
  src/tactile_feedback_node.cpp
  src/Serial.cpp
)

## Declare a C++ executable
add_executable(tactile_feedback_node ${SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(tactile_feedback_node
   ${catkin_LIBRARIES}
)