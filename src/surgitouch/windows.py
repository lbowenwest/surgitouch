import rospy
from geometry_msgs.msg import Pose2D, Vector3

import numpy as np

import pygame
from PyQt4 import QtGui, QtCore
from shapely.geometry import Point


class VisualiserWidget(QtGui.QWidget):
    def __init__(self, parent=None, zone=None, size=(600, 600), bg_color=(50, 50, 50), cursor_color=(245, 210, 0), zone_color=(255, 255, 255)):
        super(VisualiserWidget, self).__init__(parent)
        self.zone = zone or Point(0, 0).buffer(0.1)
        self.size = size
        self.bg_color = bg_color
        self.cursor_color = cursor_color
        self.zone_color = zone_color
        self.surface = pygame.Surface(self.size)
        self.update_cursor(0, 0)

        self.show_zones = True
        self.show_cursor = True
        self.show_force = True

        self.draw()

    @property
    def width(self):
        return self.size[0]

    @property
    def height(self):
        return self.size[1]

    @property
    def image_data(self):
        return self.surface.get_buffer().raw

    @property
    def image(self):
        return QtGui.QImage(self.image_data, self.width, self.height, QtGui.QImage.Format_RGB32)

    @property
    def zone_edges(self):
        return [self.zone.exterior] + list(self.zone.interiors)

    @property
    def _points(self):
        return [e.interpolate(e.project(self.cursor)) for e in self.zone_edges]

    @property
    def _distances(self):
        return [e.distance(self.cursor) for e in self.zone_edges]

    @property
    def in_zone(self):
        return self.cursor.within(self.zone)

    @property
    def shortest_distance(self):
        return min(self._distances)

    @property
    def closest_point(self):
        return self._points[self._distances.index(self.shortest_distance)]

    @property
    def cursor_pos(self):
        return self.map(self.cursor.x, self.cursor.y)

    def map(self, x, y):
        return (int((x + 1) * self.size[0]/2), int((y + 1) * self.size[1]/2))

    def update_cursor(self, x, y):
        self.cursor = Point(x, y)

    def draw_background(self):
        self.surface.fill(self.bg_color)

    def draw_cursor(self, size=2):
        pygame.draw.circle(self.surface, pygame.Color(*self.cursor_color), self.cursor_pos, size)

    def draw_zone(self):
        for edge in self.zone_edges:
            pygame.draw.polygon(self.surface, self.zone_color, [self.map(*c) for c in edge.coords], 1)

    def draw_force_line(self):
        if self.cursor.within(self.zone):
            return
        start = self.cursor_pos
        end = self.map(*np.asarray(self.closest_point.coords)[0])
        pygame.draw.line(self.window, (255, 0, 0), start, end, int(self.shortest_distance*5+1))

    def draw(self):
        self.draw_background()
        if self.show_zones:
            self.draw_zone()
        if self.show_cursor:
            self.draw_cursor()
        if self.show_force:
            self.draw_force_line()

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.drawImage(0, 0, self.image)
        qp.end()


class Visualiser(QtGui.QMainWindow):
    """docstring for Visualizer"""
    def __init__(self, zone=None, feedback=False):
        super(Visualiser, self).__init__()
        self.position_sub = rospy.Subscriber("surgitouch/position", Pose2D, self.pos_received)
        self.force_pub = rospy.Publisher("surgitouch/force", Vector3, queue_size=10)

        self.zone = zone or Point(0, 0).buffer(0.1)
        self.size = (600, 600)

        self.visualiser = VisualiserWidget(zone=self.zone, size=self.size)

        self.init_ui()

    def init_ui(self):
        self.setCentralWidget(self.visualiser)

        zoneAction = QtGui.QAction("Toggle Zones", self)
        zoneAction.triggered.connect(self.visual_toggle_zones)

        self.toolbar = self.addToolBar('Drawing Options')
        self.toolbar.addAction(zoneAction)


        # self.statusBar().showMessage('Ready')
        self.setGeometry(200, 200, *self.size)
        self.show()

    def pos_received(self, data):
        self.visualiser.update_cursor(data.x, data.y)
        self.visualiser.draw()

    def visual_toggle_zones(self):
        self.visualiser.show_zones = not self.visualiser.show_zones
        self.visualiser.draw()



# class Visualizer(object):
#     """Base class for a visualizer"""
#     def __init__(self, size=(600, 600), zone=None, background=(50,50,50), caption=""):
#         self.size = size
#         self.background = pygame.Color(*background)
#         self.caption = caption
#         self.zone = zone or Point(0, 0).buffer(0.1)
#         self.update_cursor(0,0)
#         self.window = self.init_window()
#
#     def map_to_screen(self, p):
#         return (int(self.size[0]/2 + (self.size[0]/2) * p[0]), int(self.size[1]/2 - (self.size[1]/2) * p[1]))

#     def map_to_joystick(self, p):
#         return ((p[0] - self.size[0]/2.0) / self.size/2.0, (p[1] + self.size[1]/2.0) / self.size[1]/2.0)

#     def init_window(self):
#         pygame.init()
#         window = pygame.display.set_mode(self.size)
#         if self.caption:
#             pygame.display.set_caption(self.caption)
#         window.fill(self.background)
#         pygame.display.update()
#         return window

#     def update_cursor(self, x, y):
#         self.cursor = Point(x, y)

#     @property
#     def cursor_pos(self):
#         return np.asarray(self.cursor.coords)[0]

#     @property
#     def zone_edges(self):
#         return [self.zone.exterior] + list(self.zone.interiors)

#     def draw(self):
#         self.draw_background()
#         self.draw_zone()
#         self.draw_cursor()
#         pygame.display.update()

#     def draw_background(self):
#         self.window.fill(self.background)

#     def draw_cursor(self, color=(245, 210, 0), size=2):
#         pygame.draw.circle(self.window, pygame.Color(*color), self.map_to_screen(self.cursor_pos), size)

#     def draw_zone(self):
#         for edge in self.zone_edges:
#             pygame.draw.polygon(
#                 self.window, pygame.Color(255, 255, 255),
#                 [self.map_to_screen(c) for c in edge.coords], 1)


# class Force_Visualizer(Visualizer):
#     """docstring for Force_Visualizer"""
#     def __init__(self, feedback=False, **kwargs):
#         super(Force_Visualizer, self).__init__(**kwargs)
#         self.feedback = feedback
#         # self.max_distance = max_distance or 1.0
#         self.position_sub = rospy.Subscriber("surgitouch/position", Pose2D, self.pos_received)
#         self.force_pub = rospy.Publisher("surgitouch/force", Vector3, queue_size=10)

#         self.draw()

#     def pos_received(self, data):
#         self.update_cursor(data.x, data.y)

#         x_force = self.closest_point.x - self.cursor.x if not self.in_zone else 0
#         y_force = self.closest_point.y - self.cursor.y if not self.in_zone else 0
#         print("X:{} Y:{}".format(x_force, y_force))

#         if self.feedback:
#             self.force_pub.publish(x_force, y_force, 0)

#         self.draw()


    # @property
    # def _points(self):
    #     return [e.interpolate(e.project(self.cursor)) for e in self.zone_edges]

    # @property
    # def _distances(self):
    #     return [e.distance(self.cursor) for e in self.zone_edges]

    # @property
    # def in_zone(self):
    #     return self.cursor.within(self.zone)

    # @property
    # def shortest_distance(self):
    #     return min(self._distances)

    # @property
    # def closest_point(self):
    #     return self._points[self._distances.index(self.shortest_distance)]

#     def draw_force_line(self):
#         if self.cursor.within(self.zone):
#             return
#         start = self.map_to_screen(self.cursor_pos)
#         end = self.map_to_screen(np.asarray(self.closest_point.coords)[0])
#         pygame.draw.line(self.window, (255, 0, 0), start, end, int(self.shortest_distance*5+1))

#     def draw(self, update=True):
#         self.draw_background()
#         self.draw_zone()
#         self.draw_force_line()
#         self.draw_cursor()
#         if update:
#             pygame.display.update()
