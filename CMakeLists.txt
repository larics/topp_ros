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

# Mark launch folder for installation
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark scripts folder for installation
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

# Mark all the *.xml files for installation
install(FILES package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
