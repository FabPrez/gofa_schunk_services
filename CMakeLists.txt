cmake_minimum_required(VERSION 2.8.3)
project(gofa_schunk_services)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  actionlib
  message_runtime
  message_generation
  schunk_interfaces
)

add_service_files(
  FILES
  MoveTo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gofa_schunk_services src/gofa_schunk_services_node.cpp src/gofa_schunk_services.cpp)
target_link_libraries(gofa_schunk_services ${catkin_LIBRARIES})


