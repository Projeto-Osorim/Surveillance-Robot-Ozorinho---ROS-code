cmake_minimum_required(VERSION 3.0.2)
project(img_transmission_intern)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_transmission_intern
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/captura_transmissao_imagem.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)