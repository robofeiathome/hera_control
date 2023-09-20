cmake_minimum_required(VERSION 3.0.2)
project(hera_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  tf
  message_generation
)

# catkin_python_setup()

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

#add_executable(trajectory_server src/manipulator/trajectory_server.cpp)
#add_dependencies(trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(trajectory_server ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

#add_executable(position_control src/manipulator/position_control.cpp)
#add_dependencies(position_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(position_control ${catkin_LIBRARIES})

#install(TARGETS trajectory_server position_control
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


