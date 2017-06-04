cmake_minimum_required(VERSION 2.8.3)
project(surgitouch)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS nodes/visualiser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
