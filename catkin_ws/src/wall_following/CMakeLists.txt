cmake_minimum_required(VERSION 3.0.2)
project(wall_following)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  FindWall.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs std_srvs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/find_wall.py
  scripts/wall_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)