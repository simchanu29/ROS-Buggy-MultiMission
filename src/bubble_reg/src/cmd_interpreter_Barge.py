#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

# 1 = Left
# 2 = Right

class barge_command():
    def __init__(self):
        rospy.init_node('cmd_Barge')

        self.cmdT1 = rospy.Publisher('commandT1', Float32, queue_size=1)  # 0 to 1
        self.cmdT2 = rospy.Publisher('commandT2', Float32, queue_size=1)  # 0 to 1

        self.twist_pub = rospy.Subscriber('cmd_vel', Twist, self.callback)

        self.thrustL = 0
        self.thrustR = 0

    def callback(self, msg):
        print 'Received ',msg

        twistAngZ = msg.angular.z
        twistLinX = msg.linear.x

        self.thrustL = twistLinX - twistAngZ
        self.thrustR = twistLinX + twistAngZ

        print 'Publishing ',self.thrustL,' and ',self.thrustR
        self.cmdT1.publish(self.thrustL)
        self.cmdT2.publish(self.thrustR)

if __name__ == '__main__':
    barge_command()
    rospy.spin()
