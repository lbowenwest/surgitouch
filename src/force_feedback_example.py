#!/usr/bin/env python2

# This example simulates a force feedback design
# proportional to the position, forcing it back to
# the centre

import rospy
from geometry_msgs.msg import Pose2D, Vector3


pub = rospy.Publisher('surgitouch/force', Vector3, queue_size=10)

def callback(data):
  pub.publish(-data.x, -data.y, 0)

def feedback():
  rospy.init_node('force_feedback')
  rospy.Subscriber('surgitouch/position', Pose2D, callback)
  rospy.spin()


if __name__ == '__main__':
  try:
    feedback()
  except rospy.ROSInterruptException:
    pass
