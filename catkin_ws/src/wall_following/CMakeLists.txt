cmake_minimum_required(VERSION 3.0.2)
project(ros_wall_following)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_srvs
  message_generation
  actionlib_msgs
)



add_service_files(
  FILES
  FindWall.srv
)

add_action_files(
FILES 
OdomRecord.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
  actionlib_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs nav_msgs std_srvs message_runtime actionlib_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/find_wall.py
  scripts/wall_follower.py
  scripts/record_odom_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)















