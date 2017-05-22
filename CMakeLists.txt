cmake_minimum_required(VERSION 2.4.6)
project(surgitouch)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  FILES
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
