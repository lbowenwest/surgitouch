import rospy
from geometry_msgs.msg import Pose2D, Vector3

from shapely.geometry import Point


class Simple(object):
    """ Simplest feedback method
        - Has a restoring force to origin proportional to position """
    def __init__(self):
        self.pub = rospy.Publisher("surgitouch/force", Vector3, queue_size=10)
        self.sub = rospy.Subscriber("surgitouch/position", Pose2D, self.callback)

    def callback(self, data):
        pub.publish(-data.x, -data.y, 0)


class Zone(object):
    """Zone feedback method"""
    def __init__(self, zone, max_distance=1.0):
        self.zone = zone or Point(0, 0).buffer(0.1)
        self.max_distance = max_distance

    @property
    def zone_edges(self):
        return [self.zone.exterior] + list(self.zone.interiors)

