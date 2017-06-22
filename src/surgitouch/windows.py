import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import numpy as np

import pygame
from shapely.geometry import Point


class Visualiser(object):
    """Base class for a visualizer"""
    def __init__(self, size=(600, 600), zone=None, background=(50,50,50), caption="", feedback=False):
        self.size = size
        self.background = pygame.Color(*background)
        self.caption = caption
        self.zone = zone or Point(0, 0).buffer(0.1)
        self.feedback = feedback
        self.update_cursor(0,0)
        self.window = self.init_window()
        self.sensor = True

        self._cursor = (0, 0)

        self.draw(bg=True, zone=False, cursor=False, forces=False)

        rospy.init_node("force_visuals", anonymous=True)


    def init_window(self):
        pygame.init()
        window = pygame.display.set_mode(self.size)
        if self.caption:
            pygame.display.set_caption(self.caption)
        window.fill(self.background)
        pygame.display.update()
        return window

    def init_ros(self):
        rospy.init_node("force_visuals", anonymous=True)
        self.joystick_sub = rospy.Subscriber("surgitouch/joystick", Joy, self.joy_received)
        self.force_pub = rospy.Publisher("surgitouch/force", Vector3, queue_size=10)

    def joy_received(self, data):
        self.update_cursor(*data.axes[:2])

        x_force = self.closest_point.x - self.cursor_point().x if not self.in_zone else 0
        y_force = self.closest_point.y - self.cursor_point().y if not self.in_zone else 0
        # print("X:{:4f}\tY:{:4f}\tF_x:{:4f}\tF_y:{:4f}".format(data.axes[0], data.axes[1], x_force, y_force))
        # print("X:{:4f}\tY:{:4f}".format(data.axes[0], data.axes[1]))

        # print("Speeds X:{} \t Y:{}".format(data.axes[2], data.axes[3]))

        if self.feedback:
            self.force_pub.publish(x_force, y_force, 0)

        self.draw(forces=True)

    def map_to_screen(self, p):
        return (int(self.size[0]/2 + (self.size[0]/2) * p[0]), int(self.size[1]/2 - (self.size[1]/2) * p[1]))

    def map_to_joystick(self, p):
        return ((p[0] - self.size[0]/2.0) / self.size/2.0, (p[1] + self.size[1]/2.0) / self.size[1]/2.0)

    def update_cursor(self, x, y):
        self._cursor = (x, y)
        # self.cursor = Point(x, y)

    # @property
    # def cursor_pos(self):
    #     return np.asarray(self.cursor.coords)[0]

    @property
    def cursor_pos(self):
        return self._cursor

    def cursor_point(self):
        return Point(*self.cursor_pos)

    @property
    def zone_edges(self):
        return [self.zone.exterior] + list(self.zone.interiors)

    @property
    def _points(self):
        return [e.interpolate(e.project(self.cursor_point())) for e in self.zone_edges]

    @property
    def _distances(self):
        return [e.distance(self.cursor_point()) for e in self.zone_edges]

    @property
    def in_zone(self):
        return self.cursor_point().within(self.zone)

    @property
    def shortest_distance(self):
        return min(self._distances)

    @property
    def closest_point(self):
        return self._points[self._distances.index(self.shortest_distance)]


    def draw_background(self):
        self.window.fill(self.background)

    def draw_cursor(self, color=(255, 0, 0), size=5):
        pygame.draw.circle(self.window, pygame.Color(*color), self.map_to_screen(self.cursor_pos), size, 0)

    def draw_zone(self):
        pygame.draw.polygon(self.window, pygame.Color(180, 180, 180),
            [self.map_to_screen(c) for c in self.zone.exterior.coords], 0)
        for edge in list(self.zone.interiors):
            pygame.draw.polygon(self.window, pygame.Color(*self.background),
                [self.map_to_screen(c) for c in edge.coords], 0)

    def draw_force_line(self):
        if self.cursor_point().within(self.zone):
            return
        start = self.map_to_screen(self.cursor_pos)
        end = self.map_to_screen(np.asarray(self.closest_point.coords)[0])
        pygame.draw.line(self.window, (255, 0, 0), start, end, int(self.shortest_distance*5+1))

    def draw(self, bg=True, zone=True, forces=False, cursor=True):
        if bg:
            self.draw_background()
        if zone:
            self.draw_zone()
        if forces:
            self.draw_force_line()
        if cursor:
            self.draw_cursor()
        pygame.display.update()


    def sketch_cb(self, msg):
        old_cursor = self.cursor_pos
        self.update_cursor(*msg.axes)
        if self.sensor:
            pygame.draw.line(self.window, (255, 0, 0),
                self.map_to_screen(old_cursor),
                self.map_to_screen(self.cursor_pos), 5)
            pygame.display.update()

    def sensor_cb(self, msg):
        self.sensor = msg.data


    def start(self):
        self.joystick_sub = rospy.Subscriber("surgitouch/joystick", Joy, self.joy_received)
        if self.feedback:
            self.force_pub = rospy.Publisher("surgitouch/force", Vector3, queue_size=10)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pygame.quit()

    def etch_a_sketch(self):
        self.joystick_sub = rospy.Subscriber("surgitouch/joystick", Joy, self.sketch_cb)
        # self.sensor_sub = rospy.Subscriber("surgitouch/sensor", Bool, self.sensor_cb)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pygame.quit()

    def demo(self):
        self.update_cursor(0.28, -0.22)
        self.draw(forces=True)
        try:
            while True:
                pygame.display.update()
        except KeyboardInterrupt:
            pass

