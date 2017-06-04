#!/usr/bin/env python2

"""
This starts a ROS node and a visualiser GUI to display position and force feedback
"""

import rospy
from surgitouch.windows import Visualiser

from shapely.geometry import LineString

import sys
from PyQt4 import QtGui

zone = LineString([(-0.2, -0.2), (0.2, 0.2), (-0.2, 0.2), (0.2, -0.2), (-0.2, -0,2)]).buffer(0.1)
zone = None

if __name__ == '__main__':
    rospy.init_node("force_visuals", anonymous=True)
    app = QtGui.QApplication(sys.argv)
    gui = Visualiser(zone=zone)
    app.exec_()
