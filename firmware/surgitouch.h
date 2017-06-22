#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

// define Kinematic values
#define COUNTS_PER_REV    14400
#define HEIGHT            20.5
#define CENTRE_DISTANCE   18.2
#define FORCE_MIN         0.11
#define JOYSTICK_LENGTH   110
#define TORQUE_CONST      35.2
#define NUM_STEPS         20
#define STALL_TORQUE      212
#define SAFETY_FACTOR     1.5
#define RESISTANCE        5.7
#define NOMINAL_VOLTAGE   16.0

// #define TORQUE_MIN        (FORCE_MIN * JOYSTICK_LENGTH)
#define TORQUE_MIN        (28.8)
#define TORQUE_MAX        (STALL_TORQUE / SAFETY_FACTOR)
#define TORQUE_RATIO      (TORQUE_MAX / TORQUE_MIN)

#define MAX_PWM           80

#define FORCE_THRESHOLD   0.0

#define SENSOR_THRESHOLD  20000
#define SENSOR_SEND_PIN   4
#define SENSOR_RECV_PIN   2
#define SENSOR_SAMPLES    30

// #define CURRENT_CONTROL
// #define DEBUG

#define get_pwm(val) (fabs(val) < 0.1 ? 0 : MAX_PWM * (TORQUE_MIN / (NOMINAL_VOLTAGE * TORQUE_CONST)) * RESISTANCE * pow(TORQUE_RATIO, fabs(val)))
#define get_current(val) (fabs(val) < 0.05 ? 0 : (TORQUE_MIN / TORQUE_CONST) * pow(TORQUE_RATIO, fabs(val)))

float calculate_pwm(float val);
float calculate_current(float val);
float current_to_pwm(float current);

bool get_encoder_positions(RoboClaw *rc, int32_t *enc1, int32_t *enc2);
bool get_normal_positions(RoboClaw *rc, float *x, float *y);
bool get_speeds(RoboClaw *rc, uint32_t *x, uint32_t *y);

void apply_force(RoboClaw *rc, float fx, float fy);

// subsriber callback functions
void zero_encoders(const std_msgs::Empty &message);
void force_cb(const geometry_msgs::Vector3 &message);


#endif
