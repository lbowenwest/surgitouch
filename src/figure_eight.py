#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Pose2D, Vector3

import numpy as np
from shapely.geometry import LineString, Point, Polygon

import pygame

def init_window(size, color=(0, 0, 0), caption=""):
  pygame.init()
  window = pygame.display.set_mode(size)
  if caption:
    pygame.display.set_caption(caption)
  window.fill(pygame.Color(*color))
  pygame.display.update()

  return window

SIZE = 600
def remap(x):
  return SIZE/2 + x * SIZE/2

windowObj = init_window((SIZE, SIZE), caption="Surgitouch Visualizer")
yellow = pygame.Color(245,210,0)

# distance from zone where you feel the maximum force
DISTANCE_AT_MAX_FORCE = 0.3

# coordinates describing a rotated bowtie shape
coords = [(-0.3, -0.4), (0.3, 0.4), (-0.3, 0.4), (0.3, -0.4), (-0.3, -0.4)]
line = LineString(coords)

# This is just one example of constructing a zone using shapely
# Can also use polygon and specify exterior and and any interior rings
zone = line.buffer(0.2)


# force publisher
pub = rospy.Publisher('surgitouch/force', Vector3, queue_size=10)

def draw_zone(window, zone, color=(255,255,255)):
  window.fill(pygame.Color(0,0,0))

  edges = [zone.exterior]
  for interior in list(zone.interiors):
    edges.append(interior)

  for edge in edges:
    coords = [(remap(x), remap(y)) for x,y in  edge.coords[:]]
    pygame.draw.polygon(window, pygame.Color(*color), coords, 5)

  pygame.display.update()


def get_closest_point(point, zone):
  # within the zone, so return the point
  if point.within(zone):
    return point

  # get list of all zone edges
  edges = [zone.exterior]
  for interior in zone.interiors:
    edges.append(interior)

  # Get closest point and distance for each edge
  points = []
  distances = []
  for edge in edges:
    points.append(edge.interpolate(edge.project(point)))
    distances.append(edge.distance(point))

  # get point where smallest distance and return it
  min_distance = min(distances)
  closest_point = points[distances.index(min_distance)]

  return closest_point, min_distance


def callback(data):
  # Create shapely point using position
  position = Point(data.x, data.y)

  # If we're in the zone no force feedback
  if position.within(zone):
    x_force = 0
    y_force = 0
  else:
    point, distance = get_closest_point(position, zone)

    # We want to restore to the closest point so (point - current_pos)
    x_distance = point.x - position.x
    y_distance = point.y - position.y

    # normalise
    normal_distance = distance / DISTANCE_AT_MAX_FORCE
    x_force = x_distance / normal_distance
    y_force = y_distance / normal_distance

  rospy.loginfo("X:{} Y:{} X_FORCE:{} Y_FORCE:{}".format(x_distance, y_distance, x_force, y_force))
  # pub.publish(x_force, y_force, 0)

  new_x = int((SIZE/2) + (SIZE/2)*data.x)
  new_y = int((SIZE/2) + (SIZE/2)*data.y)

  # windowObj.fill(pygame.Color(0,0,0))
  draw_zone(windowObj, zone)
  pygame.draw.circle(windowObj, yellow, (new_x, new_y), 4, 2)

  pygame.display.update()


def feedback():
  rospy.init_node('figure_eight')
  rospy.Subscriber('surgitouch/position', Pose2D, callback)
  rospy.spin()


if __name__ == '__main__':
  draw_zone(windowObj, zone)
  try:
    feedback()
  except rospy.ROSInterruptException:
    pass
