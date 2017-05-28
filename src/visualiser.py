#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Pose2D

import pygame, sys
from pygame.locals import *


def init_window(size, colour=(0, 0, 0), caption=""):
    pygame.init()
    window = pygame.display.set_mode(size)
    if caption:
        pygame.display.set_caption(caption)
    window.fill(pygame.Color(*colour))
    pygame.display.update()

    return window

WIDTH = 600
HEIGHT = 600

windowObj = init_window((WIDTH, HEIGHT), caption="Surgitouch Visualizer")

yellow = pygame.Color(245,210,0)

# Start in the middle
old_x = WIDTH / 2
old_y = HEIGHT / 2


def callback(data):
    global old_x
    global old_y

    rospy.loginfo("X: {} Y:{}".format(data.x, data.y))
    current = 0.34357 * 1.154**(data.y / 0.025)
    rospy.loginfo("[CURRENT]: {}".format(str(current)))

    new_x = int((WIDTH/2) + (WIDTH/2)*data.x)
    new_y = int((HEIGHT/2) + (HEIGHT/2)*data.y)
    windowObj.fill(pygame.Color(0,0,0))

    pygame.draw.circle(windowObj, yellow, (new_x, new_y), 2, 1)

    pygame.display.update()

    old_x = new_x
    old_y = new_y


if __name__ == '__main__':
    rospy.init_node('surgitouch_visualiser', anonymous=True)
    rospy.Subscriber("surgitouch/position", Pose2D, callback)
    rospy.spin()
