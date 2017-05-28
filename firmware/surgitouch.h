#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

// define Kinematic values
// Counts per revolution including rising and falling peaks
#define COUNTS_PER_REV    14400
// Height from centre of shaft to lid of joystick
#define HEIGHT            17.5
// Distance from centre of joystick to edge of workable space
// In this case 20.86 allows for 50 deg of movement
#define CENTRE_DISTANCE   25.6
// Minimum force obtained through testing
#define FORCE_MIN         0.11
// Length of joystick (in mm)
#define JOYSTICK_LENGTH   110
// Torque Constant (mNm)
#define TORQUE_CONST      35.2
// Stall torque of motor (mNm)
#define STALL_TORQUE      212
// Safety factor
#define SAFETY_FACTOR     1.5
// Steps
#define NUM_STEPS         20
// Resistance (Ohms)
#define RESISTANCE        3.99
// Nominal Voltage = 24V
#define NOMINAL_VOLTAGE   16.0

#define TORQUE_MIN        FORCE_MIN * JOYSTICK_LENGTH
#define TORQUE_MAX        STALL_TORQUE / SAFETY_FACTOR

#define get_PWM(length) (127 * (TORQUE_MIN / (NOMINAL_VOLTAGE * TORQUE_CONST)) * RESISTANCE * pow(TORQUE_MAX` /TORQUE_MIN, fabs(length)))

bool get_encoder_positions(RoboClaw *rc, int32_t *enc1, int32_t *enc2);
bool get_normal_positions(RoboClaw *rc, float *x, float *y);
void apply_force(RoboClaw *rc, float fx, float fy);

#ifdef TESTING
void message_cb(const std_msgs::String &message);
#endif

void force_cb(const geometry_msgs::Vector3 &message);

#endif