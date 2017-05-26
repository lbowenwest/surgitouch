#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "surgitouch.h"
#include "RoboClaw.h"


ros::NodeHandle nh;

char buffer[50] = "hello world!\0";
float x_position, y_position, x_force, y_force;

std_msgs::String str_msg;
geometry_msgs::Pose2D pos_msg;

ros::Publisher position("surgitouch/position", &pos_msg);
ros::Subscriber<geometry_msgs::Vector3> force("surgitouch/force", force_cb);

ros::Publisher chatter("surgitouch/chatter", &str_msg);
ros::Subscriber<std_msgs::String> change("surgitouch/change_message", message_cb);

void force_cb(const geometry_msgs::Vector3 &message) {
  x_force = message.x;
  y_force = message.y;
}

void message_cb(const std_msgs::String &message) {
  strcpy(buffer, message.data);
}


void setup() {
  nh.initNode();

  nh.advertise(chatter);
  nh.subscribe(change);

  nh.advertise(position);
  nh.subscribe(force);
}

void loop() {
  str_msg.data = buffer;
  chatter.publish(&str_msg);

  nh.spinOnce();

  delay(500);
}
