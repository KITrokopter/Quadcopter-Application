#!/usr/bin/env python2.7
import rospy
import time
import threading
import logging

from quadcopter_application.msg import quadcopter_controll

logger = logging.getLogger()

INITIAL_ROLL = 0.0
INITIAL_PITCH = 0.0
INITIAL_YAW = 0
INITIAL_THRUST = 10001

class CrazyDemo(object):

    def start(self):
        import sys
        factor = 1000
        thrust = INITIAL_THRUST
        roll = INITIAL_ROLL
        pitch = INITIAL_PITCH
        yaw = INITIAL_YAW
        pub = rospy.Publisher('quadcopter_controll_0', quadcopter_controll)
        rospy.init_node('crazyflie_sender')

        while not rospy.is_shutdown():
            ch = sys.stdin.read(1)    
            if ch == "a":
                thrust = thrust + factor
            elif ch == "y":
                thrust = pitch - factor
            elif ch == "s":
                thrust = roll + 1
            elif ch == "x":
                thrust = roll - 1
            elif ch == "d":
                thrust = pitch + 1
            elif ch == "c":
                thrust = pitch - 1   
            elif ch == "f":
                thrust = yaw + 1
            elif ch == "v":
                thrust = yaw - 1    
            elif ch == "e":
                thrust = 0
            else:
                thrust = thrust;
            rospy.loginfo(str)
            pub.publish(0, thrust, roll, pitch, yaw)
            rospy.sleep(0.05)

c = CrazyDemo()
c.start()
time.sleep(1)
c.stop()


