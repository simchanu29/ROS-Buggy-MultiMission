#!/usr/bin/env python
# coding=utf-8

# Author : Simon
# Notes  : Pour l'instant on utilise du float64
#          Passer les nodes en C++ si on a le temps
#          Le robot considéré ici est la barge avec 2 moteurs (1 droit et 1 gauche)
# Input  : Reçois les commandes des autres noeuds pour les moteurs
# Output : Envoie les commandes aux actuateurs
import rospy
from std_msgs.msg import Float32
from maestro.maestro import Controller

# 1000 : marche avant
# 1500 : statique
# 2000 : marche arrière
# Channel : 0:L 1:R

def callbackT1(msg):
    
    thrust1 = msg.data # Left -1;1
    rospy.loginfo("Suscriber T1 received : %d", int(thrust1*700+6000))
    maestro.setTarget(0,int(-thrust1*2000+6000))

def callbackT2(msg):

    thrust2 = msg.data # Right -1;1
    rospy.loginfo("Suscriber T2 received : %d", int(thrust2*700+6000))
    maestro.setTarget(1,int(-thrust2*2000+6000))

if __name__ == '__main__':

    rospy.init_node('driver_maestro')

    rospy.loginfo("driver_maestro Node Initialised")

    maestro = Controller()
    maestro.setAccel(0,10)
    maestro.setAccel(1,10)
    subT1 = rospy.Subscriber('commandT1', Float32, callbackT1)
    subT2 = rospy.Subscriber('commandT2', Float32, callbackT2)

    while not rospy.is_shutdown():
        try:
            rospy.rostime.wallsleep(0.5)
        except rospy.ROSInterruptException:
            maestro.close()