#ifndef _ROS_H
#define _ROS_H

#include "ros/node_handle.h"
#include "Atmega32u4Hardware.h"

namespace ros {
  typedef ros::NodeHandle_<Atmega32u4Hardware> NodeHandle;
}

#endif
