cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)

SET(CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -fno-threadsafe-statics -Wno-dev -DTESTING=1"
)

register_hardware_platform(/usr/share/arduino/hardware/archlinux-arduino/avr)

link_directories(../lib)

generate_arduino_firmware(surgitouch
  SRCS surgitouch.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD leonardo
  PORT /dev/ttyACM0
)

