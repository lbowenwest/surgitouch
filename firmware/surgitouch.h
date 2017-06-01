#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

// define Kinematic values
#define COUNTS_PER_REV    14400
#define HEIGHT            17.5
#define CENTRE_DISTANCE   25.6
#define FORCE_MIN         0.11
#define JOYSTICK_LENGTH   110
#define TORQUE_CONST      35.2
#define STALL_TORQUE      212
#define SAFETY_FACTOR     1.5
#define RESISTANCE        3.99
#define NOMINAL_VOLTAGE   16.0

#define TORQUE_MIN        FORCE_MIN * JOYSTICK_LENGTH
#define TORQUE_MAX        STALL_TORQUE / SAFETY_FACTOR
#define TORQUE_RATIO      TORQUE_MAX / TORQUE_MIN
#define TORQUE_FACTOR     127 * (TORQUE_MIN / (NOMINAL_VOLTAGE * TORQUE_CONST)) * RESISTANCE

#define MOTOR_FORWARDS    0x10
#define MOTOR_BACKWARDS   0x11


float calculate_pwm(float val);

bool get_encoder_positions(RoboClaw *rc, int32_t *enc1, int32_t *enc2);
bool get_normal_positions(RoboClaw *rc, float *x, float *y);
void apply_current(RoboClaw *rc, uint8_t x_direction, uint8_t x_val, uint8_t y_direction, uint8_t y_val);

void zero_encoders(const std_msgs::Empty &message);
void force_cb(const geometry_msgs::Vector3 &message);


#endif
