#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import Float32, Float64, Int8
#from sensor_msgs import Imu, NavSatFix
import numpy as np
import math
import matplotlib.pyplot as plt
import tf.transformations as tf
from models.Boat import Boat
from models.BlackBox import BlackBox

# 1 = Left
# 2 = Right

class world():
    def __init__(self):
        rospy.init_node('display_python')
        print "Node initialisation"

        # Subscriber
        self.TL_sub = rospy.Subscriber('commandT2', Float32, self.updateTL)
        self.TR_sub = rospy.Subscriber('commandT1', Float32, self.updateTR)
        self.state_sub = rospy.Subscriber('cmd_state', Int8, self.updateState)

        # Publisher
        self.pose_pub = rospy.Publisher('pose_real', Pose, queue_size=1)
        self.twist_pub = rospy.Publisher('twist_real', Twist, queue_size=1)
        self.pose_blackbox = rospy.Publisher('pose_blackBox', Point, queue_size=1)

        #self.imu_pub = rospy.Publisher('imu/imu', Pose, queue_size=1)
        #self.gps_pub = rospy.Publisher('pose_real', Pose, queue_size=1)
        self.angle_pub = rospy.Publisher('angle_ping', Float64, queue_size=1)

        # Internal variable
        self.dt = 0.1
        self.detectedAngle = -3
        print "Creating boat"
        self.boat = Boat(0,0,0,0,0,0)
        self.blackBox = BlackBox(20,18,10)

    def updateTL(self, msg):
        # print 'received TL : ',msg
        self.boat.TL = msg.data

    def updateTR(self, msg):
        # print 'received TR : ',msg
        self.boat.TR = msg.data

    def updateState(self, msg):
        self.boat.state = msg.data

    def updateWorld(self):
        coeffFrot = 10
        dx,dy,dtheta,dv = self.boat.move(coeffFrot)

        # print 'dx : ',dx,dy,dtheta,dv

        self.boat.update(
            self.boat.x     + (dx    )*self.dt,
            self.boat.y     + (dy    )*self.dt,
            self.boat.theta + (dtheta)*self.dt,
            self.boat.v     + (dv    )*self.dt,
            dtheta
        )


        self.getAngleBlackBox()

    def getAngleBlackBox(self):

        # Si le bateau est assez près ET que son cap est à moins de 90 degré de la boite noire
        dist2BlackBox = np.sqrt((self.blackBox.x - self.boat.x)**2 + (self.blackBox.y - self.boat.y)**2)
        print 'dist2BlackBox = ',dist2BlackBox
        angle2BlackBox = self.angle_add(math.atan2(self.blackBox.y - self.boat.y,self.blackBox.x - self.boat.x),-self.boat.theta)
        print 'angle2BlackBox = ',angle2BlackBox/np.pi*180.0,' deg'

        # print '-self.boat.theta = ',-self.boat.theta,' deg'
        # print 'self.angle_add(angle2BlackBox,-self.boat.theta) = ',self.angle_add(0,0)/np.pi*180.0,' deg'
        # print 'np.abs(self.angle_add(angle2BlackBox,-self.boat.theta)) = ',np.abs(self.angle_add(angle2BlackBox,-self.boat.theta))/np.pi*180.0,' deg'

        if dist2BlackBox<self.blackBox.range \
                and np.abs(self.angle_add(angle2BlackBox,-self.boat.theta))<np.pi/2.0:

            self.detectedAngle = angle2BlackBox
        else:
            self.detectedAngle = -3
        print 'Attribute angle : ',self.detectedAngle

    def angle_add(self, a1, a2):
        return np.mod(( a1 + a2 + 3*np.pi), 2*np.pi) - np.pi

    def spin(self):

        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():

            self.updateWorld()

            # print 'Pose : ', self.boat.pose
            # print 'self.boat.pose.orientation.x : ',self.boat.pose.orientation.x
            # print 'self.boat.pose.orientation.y : ',self.boat.pose.orientation.y
            # print 'self.boat.pose.orientation.z : ',self.boat.pose.orientation.z
            # print 'self.boat.pose.orientation.w : ',self.boat.pose.orientation.w
            # print 'self.boat.pose.position.x : ',self.boat.pose.position.x
            # print 'self.boat.pose.position.y : ',self.boat.pose.position.y
            # print 'self.boat.pose.position.z : ',self.boat.pose.position.z

            self.pose_pub.publish(self.boat.pose)
            self.twist_pub.publish(self.boat.twist)

            if self.boat.state != 0:
                print "Publishing detectedAngle : ",self.detectedAngle
                self.angle_pub.publish(self.detectedAngle)
            self.pose_blackbox.publish(self.blackBox.point)

            rate.sleep()


if __name__ == '__main__':
    print "Node created"
    w = world()
    print "Spinning"
    w.spin()
