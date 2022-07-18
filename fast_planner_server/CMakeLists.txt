cmake_minimum_required(VERSION 3.0.2)
project(fast_planner_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  genmsg
  actionlib
  actionlib_msgs
)

add_action_files(
  DIRECTORY
  action
  FILES
  FastPlanner.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/fast_planner_server_node.py
  scripts/fast_planner_client_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)