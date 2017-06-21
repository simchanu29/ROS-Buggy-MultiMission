# coding=utf-8
import numpy as np
from geometry_msgs.msg import Point
import tf.transformations as tf

class BlackBox():

    def __init__(self,x,y,range):
        self.x = x
        self.y = y
        self.point = Point(x,y,0.0)
        self.range = range
