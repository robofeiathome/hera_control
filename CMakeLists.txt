cmake_minimum_required(VERSION 3.0.2)
project(hera_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf
)

# catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Manip3_Goal.msg
  Head_Goal.msg
)

add_service_files(
  FILES
  Manip3.srv
  Manip_poses.srv
  Head_service.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)



