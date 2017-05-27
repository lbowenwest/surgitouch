#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#ifdef TESTING
#include <std_msgs/String.h>
#endif

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

#include "surgitouch.h"

// declare ROS node handler
ros::NodeHandle nh;

// declare RoboClaw
// Serial1 is hardware serial on leonardo
RoboClaw rc(&Serial1, 10000);
#define address 0x80

float torque_min = FORCE_MIN * JOYSTICK_LENGTH;
float torque_max = STALL_TORQUE / SAFETY_FACTOR;
float x = pow((torque_max / torque_min), (1 / NUM_STEPS));

// values to store current positions, and desired forces
float x_position, y_position, x_force, y_force;

// message to send position to ROS
geometry_msgs::Pose2D pos_msg;

// publisher for position
ros::Publisher position("surgitouch/position", &pos_msg);
// subscriber to force changes
ros::Subscriber<geometry_msgs::Vector3> force("surgitouch/force", force_cb);

// store forces in global variables
void force_cb(const geometry_msgs::Vector3 &message) {
  x_force = message.x;
  y_force = message.y;
}

#ifdef TESTING
// buffer to store text, used in testing
char buffer[50] = "hello world!\0";
std_msgs::String str_msg;

ros::Publisher chatter("surgitouch/chatter", &str_msg);
ros::Subscriber<std_msgs::String> change("surgitouch/change_message", message_cb);

void message_cb(const std_msgs::String &message) {
  strcpy(buffer, message.data);
}
#endif


void setup() {
  // initialise node handler
  nh.initNode();

#ifdef TESTING
  nh.advertise(chatter);
  nh.subscribe(change);
#endif

  // advertise position channel, and subscribe to force channel
  nh.advertise(position);
  nh.subscribe(force);

  // initialise RoboClaw communication
  rc.begin(38400);
  // set initial encoder values to zero
  rc.SetEncM1(address, 0);
  rc.SetEncM2(address, 0);
}


void loop() {
  // declare variable for positions, then read them
  float pos_x, pos_y;
  bool valid = get_normal_positions(&rc, &pos_x, &pos_y);

  // if we got a valid result, publish the results
  if (valid) {
    pos_msg.x = pos_x;
    pos_msg.y = pos_y;

    position.publish(&pos_msg);
  }


#ifdef TESTING
  str_msg.data = buffer;
  chatter.publish(&str_msg);
#endif

  nh.spinOnce();

  delay(100);

}

bool get_encoder_positions(RoboClaw *rc, int32_t *enc1, int32_t *enc2) {
  // variables to hold status info and if valid read
  uint8_t status1, status2;
  bool valid1, valid2;

  // read encoder values using passed in RoboClaw
  *enc1 = rc->ReadEncM1(address, &status1, &valid1);
  *enc2 = rc->ReadEncM2(address, &status2, &valid2);

  // if valid reads return true
  if (valid1 & valid2)
    return true;
  else
    return false;
}

bool get_normal_positions(RoboClaw *rc, float *x, float *y) {
  // store encoder values
  int32_t enc1, enc2;
  bool valid = get_encoder_positions(rc, &enc1, &enc2);

  // if we didn't get a valid read fail out
  if (!valid)
    return false;

  // calculate the relevant lengths for x and y
  float length_x = HEIGHT * tan(((float)enc1 * 6.2832) / COUNTS_PER_REV);
  float length_y = HEIGHT * tan(((float)enc2 * 6.2832) / COUNTS_PER_REV);

  // normalise these values, and constrain them between -1 and 1
  *x = constrain(length_x / CENTRE_DISTANCE, -1, 1);
  *y = constrain(length_y / CENTRE_DISTANCE, -1, 1);

  return true;
}