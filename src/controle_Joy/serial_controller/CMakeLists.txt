cmake_minimum_required(VERSION 3.0.2)
project(serial_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(teste src/joy.cpp)
target_link_libraries(teste ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/joy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)