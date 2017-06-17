#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Wrench


class key_interpreter():
    def __init__(self):
        # [linx, liny, ang]
        self.key_mapping = {'z': [1, 0, 0], 's': [-1, 0, 0],
                            'q': [0, 1, 0], 'd': [0, -1, 0],
                            'a': [0, 0, 1], 'e': [0, 0, -1],
                            ' ': ['STOP', 'STOP'],
                            'r': ['BACK', 'BACK'],

                            '8': [1, 0, 0], '2': [-1, 0, 0],
                            '4': [0, 1, 0], '6': [0, -1, 0],
                            '7': [0, 0, 1], '9': [0, 0, -1],
                            '5': ['STOP', 'STOP'],
                            '0': ['BACK', 'BACK'],

                            'm': ['s', 'manual'], 'l': ['s', 'linefollowing'], 'k': ['s', 'stationkeeping']}

        self.linCmd = [0.0,0.0]
        self.angCmd = 0

        self.stateMap = {'manual': 0,
                         'linefollowing': 1,
                         'stationkeeping': 2}
        self.state = self.stateMap['manual']

        self.slowRate = 5  # en pourdixmille diminué à chaque pas
        self.max_torque = rospy.get_param('max_torque', 10.0)  # N.m
        self.max_force = rospy.get_param('max_force', 10.0)  # N

    def keys_cb(self, msg, twist_pub):

        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data):
            print 'len(msg.data) == 0 : ', len(msg.data) == 0
            print 'self.key_mapping.has_key(msg.data) : ', self.key_mapping.has_key(msg.data)
            print 'unknown key : ', msg.data
            return  # unknown key

        vals = self.key_mapping[msg.data]
        print "received mgs.data : ", msg.data
        print 'vals : ', vals

        # Gestion des entrées
        if isinstance(vals[0], int):

            self.angCmd += vals[2] / 20.0
            self.linCmd[0] += vals[0] / 20.0
            self.linCmd[1] += vals[1] / 20.0

            print 'self.angCmd : ', self.angCmd
            print 'self.linCmd : ', self.linCmd

            print 'self.linCmd : ', self.linCmd
            print 'self.angCmd : ', self.angCmd

        elif vals[0] == 'STOP':
            self.angCmd = 0.0
            self.linCmd = [0.0,0.0]

        elif vals[0] == 'BACK':
            if self.angCmd != 0.0:
                self.angCmd = max(min(-self.angCmd / abs(self.angCmd), 1), -1)
            for i in range(len(self.linCmd)):
                if self.linCmd[i] != 0.0:
                    self.linCmd[i] = max(min(-self.linCmd[i] / abs(self.linCmd[i]), 1), -1)

        elif vals[0] == 's':
            self.state = self.stateMap[vals[1]]
            print 'cmd state : ', vals[1]
            
        if self.state==self.stateMap['manual']:
            # Bornage
            self.angCmd = max(min(self.angCmd, 1), -1)
            self.linCmd[1] = max(min(self.linCmd[0], 1), -1)
            self.linCmd[0] = max(min(self.linCmd[1], 1), -1)

            # Creation du message twist
            print 'cmd motors :  angular[', self.angCmd, '] / linear', self.linCmd
            t = Wrench()

            t.torque.x = 0
            t.torque.y = 0
            t.torque.z = self.angCmd * self.max_torque

            t.force.x = self.linCmd[0] * self.max_force
            t.force.y = self.linCmd[1] * self.max_force
            t.force.z = 0

            # Publish
            twist_pub.publish(t)

        state_pub.publish(self.state)


if __name__ == '__main__':
    ki = key_interpreter()
    rospy.init_node('keys_to_wrench')

    twist_pub = rospy.Publisher('cmd_wrench', Wrench, queue_size=1)
    state_pub = rospy.Publisher('cmd_state', Int8, queue_size=1)

    rospy.Subscriber('keys', String, ki.keys_cb, twist_pub)

    rospy.spin()
