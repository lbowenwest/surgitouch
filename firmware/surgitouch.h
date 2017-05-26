#ifndef _SURGITOUCH_H
#define _SURGITOUCH_H

void message_cb(const std_msgs::String &message);
void force_cb(const geometry_msgs::Vector3 &message);

#endif