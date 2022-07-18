cmake_minimum_required(VERSION 3.0.2)
project(autotakeoff)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  message_generation
)

add_service_files(
  FILES
  Takeoff.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_remapper_node.py
  scripts/pos_cmd_remapper.py
  scripts/takeoff_node.py
  scripts/takeoff_server.py
  scripts/square_flight_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
