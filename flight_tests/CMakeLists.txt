cmake_minimum_required(VERSION 3.0.2)
project(flight_tests)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  # scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
