#include "surgitouch.h"

#include "ros.h"
#include "Atmega32u4Hardware.h"

#include "roboclaw.h"

// Include C headers (ie, non C++ headers) in this block
extern "C" {
#ifdef TESTING
  #include <string.h>
#endif
  #include <avr/io.h>
  #include <util/delay.h>
  #include <LUFA/Drivers/USB/USB.h>
}

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}


ros::NodeHandle nh;

float x_position, y_position, x_force, y_force;

geometry_msgs::Pose2D pos_msg;

ros::Publisher position("surgitouch/position", &pos_msg);
ros::Subscriber<geometry_msgs::Vector3> force("surgitouch/force", force_cb);

void force_cb(const geometry_msgs::Vector3 &message) {
  x_force = message.x;
  y_force = message.y;
}

#ifdef TESTING
std_msgs::String str_msg;
ros::Publisher chatter("surgitouch/chatter", &str_msg);
ros::Subscriber<std_msgs::String> change("surgitouch/change_message", message_cb);

void message_cb(const std_msgs::String &message) {
  strcpy(buffer, message.data);
}

#endif



int main() {
  uint32_t last_time = 0UL; // store time

  // Initialise Roboclaw
  rc_init();

  // Initialise ROS
  nh.initNode();

#ifdef TESTING
  nh.advertise(chatter);
  nh.subscribe(change);
#endif

  // Actual topics
  nh.advertise(position);
  nh.subscribe(force);

  while(1) {
    // Send the message every second
    if(time_now() - last_time > WAIT_TIME) {

      int16_t current1 = 0, current2 = 0;
      pos_msg.theta = ReadCurrents(current1, current2);
      pos_msg.x = current1; pos_msg.y = current2;

      position.publish(&pos_msg);

#ifdef TESTING
      str_msg.data = buffer;
      chatter.publish(&str_msg);
#endif

      last_time = time_now();
    }

    nh.spinOnce();

    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

  return 0;
}
