#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

#define TESTING 1

#define WAIT_TIME 10

#ifdef TESTING
#define WAIT_TIME 1000
#include "std_msgs/String.h"
char buffer[50] = "hello world!\0";
void message_cb(const std_msgs::String &message);
#endif

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"

void force_cb(const geometry_msgs::Vector3 &message);


#endif