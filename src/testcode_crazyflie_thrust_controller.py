#!/usr/bin/env python2.7
import rospy
import time
import threading
import logging

from quadcopter_application.msg import quadcopter_controll

logger = logging.getLogger()

INITIAL_ROLL = 0.0
INITIAL_PITCH = 0.0
INITIAL_YAWRATE = 0
INITIAL_THRUST = 10001

class CrazyDemo(object):

    def start(self):
        import sys
        factor = 1000
        thrust = INITIAL_THRUST
        pub = rospy.Publisher('quadcopter_controll_0', quadcopter_controll)
        rospy.init_node('crazyflie_sender')

        while not rospy.is_shutdown():
            ch = sys.stdin.read(1)    
            if ch == "u":
                thrust = thrust + factor
            elif ch == "d":
                thrust = thrust - factor
            elif ch == "e":
                thrust = 0
            else:
                thrust = thrust;
            rospy.loginfo(str)
            pub.publish(0, thrust, 0.0, 0.0, 0.0)
            rospy.sleep(0.05)

c = CrazyDemo()
c.start()
time.sleep(1)
c.stop()


