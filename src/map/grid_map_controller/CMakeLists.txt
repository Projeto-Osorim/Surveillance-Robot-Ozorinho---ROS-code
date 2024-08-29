cmake_minimum_required(VERSION 3.0.2)
project(grid_map_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grid_map_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/teste.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

