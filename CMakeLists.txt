cmake_minimum_required(VERSION 3.0.2)
project(hera_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  kdl_parser
  pluginlib
  urdf
  geometry_msgs
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  tf
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Manip_Goal.msg
  Joint_Goal.msg
)

add_service_files(
  FILES
  Manip_service.srv
  Joint_service.srv
  Furniture.srv
  Raposo_service.srv
  Look_for_person.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  hera_objects
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs moveit_msgs  moveit_core moveit_ros_planning moveit_ros_planning_interface message_runtime dynamixel_workbench_msgs dynamixel_workbench_toolbox
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)