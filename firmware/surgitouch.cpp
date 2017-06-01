#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <PID_v1.h>

#include "surgitouch.h"

// declare ROS node handler
ros::NodeHandle nh;

// declare RoboClaw, Serial1 is hardware serial on leonardo
RoboClaw rc(&Serial1, 10000);
#define address 0x80


// PID current control for each motor
double Kp = 0, Ki = 0, Kd = 0;
uint8_t x_direction = MOTOR_FORWARDS, y_direction = MOTOR_FORWARDS;
// PID control for x motor
double x_setpoint = 0, x_input = 0, x_output = 0;
PID x_control(&x_setpoint, &x_input, &x_output, Kp, Ki, Kd, DIRECT);
// PID control for y motor
double y_setpoint = 0, y_input = 0, y_output = 0;
PID y_control(&y_setpoint, &y_input, &y_output, Kp, Ki, Kd, DIRECT);


// message to send position to ROS
geometry_msgs::Pose2D pos_msg;
// publisher for position
ros::Publisher position("surgitouch/position", &pos_msg);


// subscriber to force changes
ros::Subscriber<geometry_msgs::Vector3> force("surgitouch/force", force_cb);
// store forces in global variables
void force_cb(const geometry_msgs::Vector3 &message) {
  x_direction = message.x > 0 ? MOTOR_FORWARDS : MOTOR_BACKWARDS;
  x_setpoint = calculate_pwm(message.x);
  y_direction = message.y > 0 ? MOTOR_FORWARDS : MOTOR_BACKWARDS;
  y_setpoint = calculate_pwm(message.y);
}


// subscriber to zero encoders
ros::Subscriber<std_msgs::Empty> zero("surgitouch/zero", zero_encoders);
// zero the encoders
void zero_encoders(const std_msgs::Empty &mesage) {
  rc.SetEncM1(address, 0);
  rc.SetEncM2(address, 0);
}


// setup function, runs once
void setup() {
  // initialise node handler
  nh.initNode();

  // advertise position channel, and subscribe to force and zero channels
  nh.advertise(position);
  nh.subscribe(force);
  nh.subscribe(zero);

  // initialise RoboClaw communication
  rc.begin(38400);
  // set initial encoder values to zero
  rc.SetEncM1(address, 0);
  rc.SetEncM2(address, 0);

  // set output limits for current controllers
  x_control.SetOutputLimits(0, 127);
  y_control.SetOutputLimits(0, 127);
}

// loop function, runs forever
void loop() {
  // declare variable for positions, then read them
  float x_pos, y_pos;
  get_normal_positions(&rc, &x_pos, &y_pos);

  // publish the positions
  pos_msg.x = x_pos;
  pos_msg.y = y_pos;

  position.publish(&pos_msg);

  // read PWM values and set as current control inputs
  x_input = rc.ReadSpeedM1(address);
  y_input = rc.ReadSpeedM2(address);
  // compute new current control
  x_control.Compute();
  y_control.Compute();
  // apply the currents through the roboclaw
  apply_current(&rc, x_direction, (uint8_t)x_output, y_direction, (uint8_t)y_output);

  // spin the node
  nh.spinOnce();
}

float calculate_pwm(float val) {
  float f = fabs(val);
  if (f < 0.1)
    return 0;
  return TORQUE_FACTOR * pow(TORQUE_RATIO, f);
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

void apply_current(RoboClaw *rc, uint8_t x_direction, uint8_t x_val, uint8_t y_direction, uint8_t y_val) {
  // switch (x_direction) {
  //   case MOTOR_FORWARDS:
  //     rc->ForwardM1(address, x_val);
  //     break;
  //   case MOTOR_BACKWARDS:
  //     rc->BackwardM1(address, x_val);
  //     break;
  // }

  switch (y_direction) {
    case MOTOR_FORWARDS:
      rc->ForwardM2(address, y_val);
      break;
    case MOTOR_BACKWARDS:
      rc->BackwardM2(address, y_val);
      break;
  }
}
