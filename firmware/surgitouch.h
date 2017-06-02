#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

// define Kinematic values
#define COUNTS_PER_REV    14400
#define HEIGHT            17.5
#define CENTRE_DISTANCE   25.6
#define FORCE_MIN         0.11
#define JOYSTICK_LENGTH   110
#define TORQUE_CONST      35.2
#define NUM_STEPS         20
#define STALL_TORQUE      212
#define SAFETY_FACTOR     1.5
#define RESISTANCE        3.99
#define NOMINAL_VOLTAGE   16.0
#define MIN_CURRENT       0.0

// #define TORQUE_MIN        (FORCE_MIN * JOYSTICK_LENGTH)
#define TORQUE_MIN        (28.8)
#define TORQUE_MAX        (STALL_TORQUE / SAFETY_FACTOR)
#define TORQUE_RATIO      (TORQUE_MAX / TORQUE_MIN)

#define MAX_PWM           64

#define FORCE_THRESHOLD   0.1

#define CURRENT_CONTROL

#define get_pwm(val) (MAX_PWM * (TORQUE_MIN / (NOMINAL_VOLTAGE * TORQUE_CONST)) * RESISTANCE * pow(TORQUE_RATIO, fabs(val)))
#define get_current(val) ((TORQUE_MIN / TORQUE_CONST) * pow(TORQUE_RATIO, fabs(val)*NUM_STEPS) - MIN_CURRENT)


bool get_encoder_positions(RoboClaw *rc, int32_t *enc1, int32_t *enc2);
bool get_normal_positions(RoboClaw *rc, float *x, float *y);

void apply_current(RoboClaw *rc, double x_current, double y_current);
void apply_force(RoboClaw *rc, float fx, float fy);

void zero_encoders(const std_msgs::Empty &message);
void force_cb(const geometry_msgs::Vector3 &message);


#endif
