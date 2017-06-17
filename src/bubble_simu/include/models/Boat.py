# coding=utf-8
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import tf.transformations as tf

class Boat():

    def __init__(self,x=0,y=0,theta=0,v=0,delta=0,TL=0,TR=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.delta = delta

        self.TL = TL
        self.TR = TR

        self.state = 0

        self.coeffMotor = 5
        self.distMotor = 0.3
        self.mass = 10
        self.inertieRot = 30

        self.pose = Pose()
        self.twist = Twist()

        # Initialisation de la Pose
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = 0
        q = tf.quaternion_from_euler(0, 0, theta)
        self.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # Initialisation de la Twist
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0

        print "Boat created"


    def move(boat,coeffFrot):
        # print 'prev TL :',boat.TL
        # print 'prev TR :',boat.TR
        # print 'prev v :',boat.v

        # on repart


        # Dans un repère absolu

        dx = boat.v * np.cos(boat.theta)
        dy = boat.v * np.sin(boat.theta)
        dtheta = (boat.TL-boat.TR)*boat.coeffMotor/boat.distMotor/boat.inertieRot
        dv = ((boat.TL+boat.TR)*boat.coeffMotor - coeffFrot*boat.v)/boat.mass

        return dx,dy,dtheta,dv


    def update(self,x,y,theta,v,delta):
        # ce sont les coordonnées globales
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.delta = delta

        # Pose en coordonnée globales avec les angles a partir du nord
        self.pose.position.x = x
        self.pose.position.y = y
        q = tf.quaternion_from_euler(0, 0, theta)
        self.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # Twist en coordonnées locales
        self.twist.angular.z = delta
        self.twist.linear.x = v
