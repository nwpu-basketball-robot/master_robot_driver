cmake_minimum_required(VERSION 2.8.3)
project(basketball_driver)

find_package(catkin REQUIRED COMPONENTS
  basketball_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_driver
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_driver src/robot_driver.cpp)
target_link_libraries(robot_driver ${catkin_LIBRARIES})
add_dependencies(robot_driver basketball_msgs_gencpp)
