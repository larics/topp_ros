cmake_minimum_required(VERSION 2.8.3)
project(topp_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy 
  std_msgs
  trajectory_msgs
  geometry_msgs
  message_generation
  roslib)

add_service_files(
  FILES
  GenerateTrajectory.srv
  GetHelixPoints.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs trajectory_msgs geometry_msgs
)

catkin_install_python(PROGRAMS scripts/generate_toppra_trajectory.py scripts/test_toppra.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
