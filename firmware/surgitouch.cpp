#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

#include <TimedPID.h>

#include "surgitouch.h"

// declare ROS node handler
ros::NodeHandle nh;

// declare RoboClaw, Serial1 is hardware serial on leonardo
RoboClaw rc(&Serial1, 10000);
#define address 0x80

#ifdef CURRENT_CONTROL
  // PID current control for each motor
  double Kp = 0, Ki = 30, Kd = 0;
  // PID control for x motor
  TimedPID x_pid(Kp, Ki, Kd);
  // PID control for y motor
  TimedPID y_pid(Kp, Ki, Kd);
#endif

// message to send position to ROS
geometry_msgs::Pose2D pos_msg;
// publisher for position
ros::Publisher position("surgitouch/position", &pos_msg);


// subscriber to force changes
ros::Subscriber<geometry_msgs::Vector3> force("surgitouch/force", force_cb);
// store forces in global variables
float x_force = 0, y_force = 0;
void force_cb(const geometry_msgs::Vector3 &message) {
  x_force = message.x;
  y_force = message.y;
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

#ifdef CURRENT_CONTROL
  // Set current output limits for safety
  x_pid.setCmdRange(0, MAX_PWM);
  y_pid.setCmdRange(0, MAX_PWM);
#endif
}

// loop function, runs forever
void loop() {
  // declare variables for positions, then read them
  float x_pos, y_pos;
  get_normal_positions(&rc, &x_pos, &y_pos);

  // publish the positions
  pos_msg.x = x_pos;
  pos_msg.y = y_pos;

  position.publish(&pos_msg);

#ifdef CURRENT_CONTROL
  // calculate current setpoint from force values, and calculate PWM setpoint
  float x_setpoint = get_current(x_force) * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE;
  float y_setpoint = get_current(y_force) * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE;

  // read current values and set as control inputs
  int16_t x_current_rc, y_current_rc;
  rc.ReadCurrents(address, x_current_rc, y_current_rc);
  float x_current = (float)x_current_rc / 100;
  float y_current = (float)y_current_rc / 100;
  // float x_input = (float)x_current * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE;
  // float y_input = (float)y_current * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE;
  float x_input = current_to_pwm(x_current);
  float y_input = current_to_pwm(y_current);
  // calculate new values
  float x_output = x_pid.getCmd(x_setpoint, x_input);
  float y_output = y_pid.getCmd(y_setpoint, y_input);

#else
  float x_output = calculate_pwm(x_force);
  float y_output = calculate_pwm(y_force);
#endif

  apply_force(&rc, x_output, y_output);

  // spin the node
  nh.spinOnce();
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


float calculate_pwm(float val) {
  float f = fabs(val);
  if (f < FORCE_THRESHOLD)
    return 0;
  return MAX_PWM * (TORQUE_MIN / (NOMINAL_VOLTAGE * TORQUE_CONST)) * RESISTANCE * pow(TORQUE_RATIO, f);
}

float calculate_current(float val) {
  float f = fabs(val);
  if (f < FORCE_THRESHOLD)
    return 0;
  return (TORQUE_MIN / TORQUE_CONST) * pow(TORQUE_RATIO, f);
}

float current_to_pwm(float current) {
  if (current < 0.8)
    return 1;
  else
    return current * RESISTANCE / NOMINAL_VOLTAGE;
}


void apply_current(RoboClaw *rc, float x_current, float y_current) {
  float pwm_x = constrain((x_current * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE), 0, MAX_PWM);
  float pwm_y = constrain((y_current * MAX_PWM * RESISTANCE / NOMINAL_VOLTAGE), 0, MAX_PWM);

  if (x_force < 0) // global force value to indicate direction
    rc->ForwardM2(address, pwm_y);
  else
    rc->BackwardM2(address, pwm_y);

  // if (y_force > 0)
  //   rc->ForwardM1(address, pwm_x);
  // else
  //   rc->BackwardM1(address, pwm_x);

}

void apply_force(RoboClaw *rc, float fx, float fy) {
  fx = constrain(fx, 0, MAX_PWM);
  fx = constrain(fx, 0, MAX_PWM);

  if (x_force < 0)
    rc->ForwardM2(address, fx);
  else
    rc->BackwardM2(address, fx);

  // if (y_force < 0)
  //   rc->ForwardM1(address, fy);
  // else
  //   rc->BackwardM1(address, fy);
}
